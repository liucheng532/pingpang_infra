"""Read-only joint feedback with physical q and separate V3 normalized features."""
import copy

from utils.joint_mapping import FROM_GYM_TO_LAB, LAB_JOINT_NAMES
from .telemetry import integer, number, vector
from .joint_normalization import JointNormalizer


ROLES = {'66': 'table_right', '198': 'table_left'}


class JointFeedback:
    """One bounded stream/session per robot. Reconnects require a new instance.

    DDS q is SDK/Gym order, stock V9 state q is already Lab order. Reordering
    does not reflect a physical robot's joints or imply training ABI acceptance.
    Robot receipt/publication timestamps remain in that robot's clock domain.
    """
    def __init__(self):
        self.normalizer = JointNormalizer()
        self.streams = {}
        self.latest = {}
        self.faults = {}

    def ingest(self, envelope):
        if not isinstance(envelope, dict):
            raise ValueError('joint_envelope_must_be_object')
        slot = envelope.get('robot_id')
        if not isinstance(slot, str) or slot not in ROLES:
            raise ValueError('unknown_robot_id')
        if slot in self.faults:
            raise ValueError('stream_fault_latched:' + self.faults[slot])
        try:
            result, identity = self._parse(envelope, slot)
            previous = self.streams.get(slot)
            if previous:
                if identity[:2] != previous[:2]:
                    raise ValueError('source_or_session_changed')
                if identity[2] <= previous[2]:
                    raise ValueError('duplicate_or_backward_sequence')
                if identity[3] <= previous[3]:
                    raise ValueError('source_clock_not_increasing')
                if identity[4] < previous[4]:
                    raise ValueError('workstation_clock_went_backwards')
            self.streams[slot] = identity
            self.latest[slot] = copy.deepcopy(result)
            return result
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.faults[slot] = str(exc)
            raise ValueError(str(exc)) from exc

    def _parse(self, envelope, slot):
        session = envelope['session_id']
        if not isinstance(session, str) or not 0 < len(session) <= 128:
            raise ValueError('invalid_session_id')
        receipt = number(envelope['received_workstation_monotonic_s'])
        if receipt < 0:
            raise ValueError('invalid_workstation_clock')
        raw = envelope['payload']
        if not isinstance(raw, dict):
            raise ValueError('joint_payload_must_be_object')
        if raw.get('kind') == 'lowstate':
            source = 'dds_sdk_lowstate'
            counter = integer(raw['tick'])
            stamp = number(raw['receive_monotonic_s'])
            q = vector(raw['q_sdk_order'], 29)
            dq = vector(raw['dq_sdk_order'], 29)
            q = [q[i] for i in FROM_GYM_TO_LAB]
            dq = [dq[i] for i in FROM_GYM_TO_LAB]
            quat = vector(raw['imu_quaternion_raw'], 4, quaternion=True)
            clock_field = 'robot_dds_receive_monotonic_s'
        elif raw.get('schema_version') == 'v9-robot-state-v1':
            source = 'stock_v9_state'
            if raw['robot'] != ROLES[slot]:
                raise ValueError('robot_role_mismatch')
            if raw.get('valid') is not True:
                raise ValueError('producer_state_invalid')
            counter = integer(raw['sequence'])
            # Preserve nanoseconds as an integer, including for ordering checks.
            stamp = integer(raw['source_monotonic_ns'], positive=True)
            q, dq = vector(raw['q'], 29), vector(raw['dq'], 29)
            quat = vector(raw['imu_quaternion_wxyz'], 4, quaternion=True)
            clock_field = 'robot_state_publication_monotonic_ns'
        else:
            raise ValueError('unsupported_joint_source')
        if stamp <= 0:
            raise ValueError('invalid_robot_clock')
        return {
            'kind': 'joint_feedback', 'robot_id': slot, 'role': ROLES[slot],
            'training_slot': 'left' if slot == '66' else 'right',
            'session_id': session, 'source': source, 'source_counter': counter,
            'source_clock': {clock_field: stamp},
            'received_workstation_monotonic_s': receipt,
            'joint_names': list(LAB_JOINT_NAMES), 'q_lab_rad': q,
            'normalized_joint_position_v3': self.normalizer.normalize(q, LAB_JOINT_NAMES).tolist(),
            'joint_normalization_urdf_sha256': self.normalizer.profile['urdf_sha256'],
            'filter_normalization_applied': True,
            'dq_lab_rad_s': dq, 'imu_quaternion_wxyz': quat,
            'joint_order_basis': 'frozen V9 joint_mapping.py and PlannerRosBridge.publish_state',
            'normalization_applied': False, 'reflection_applied': False,  # q_lab_rad remains physical
            'transport_robot_identity': 'caller_assigned; physical mapping requires independent evidence',
            'sensor_age_verified': False, 'training_joint_abi_accepted': False,
            'real_input_accepted': False, 'command_ack': False,
        }, (session, source, counter, stamp, receipt)

    def snapshot(self, now, maximum_receive_age_s=.25):
        now, timeout = number(now), number(maximum_receive_age_s)
        if timeout <= 0:
            raise ValueError('invalid_receive_timeout')
        missing, stale = [], []
        for slot in ROLES:
            if slot not in self.latest:
                missing.append(slot)
            else:
                age = now - self.latest[slot]['received_workstation_monotonic_s']
                if not 0 <= age <= timeout:
                    stale.append(slot)
        return {'kind': 'joint_feedback_pair', 'robots': copy.deepcopy(self.latest),
                'missing_robots': missing, 'stale_receive_robots': stale,
                'stream_faults': dict(self.faults),
                'both_streams_recently_received': not (missing or stale or self.faults),
                'synchronized_sample': False, 'sensor_age_verified': False,
                'real_input_accepted': False, 'command_ack': False}
