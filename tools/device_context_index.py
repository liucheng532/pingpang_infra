#!/usr/bin/env python3
"""Render human-readable context indexes from the captured device inventories."""
import json
from pathlib import Path
from device_context_ssh import ROOT, HOSTS

LABELS={'workstation':'工作站二 / Planner','right':'66 / table_right / 左手镜像','left':'198 / table_left / 右手 canonical'}


def main():
    verification_path=ROOT/'doc/device_context/verification.json'
    verification=json.loads(verification_path.read_text()) if verification_path.exists() else None
    lines=['# 四设备本地 context 索引','',
        '本开发机与三台远程设备的相关代码集中保存在本项目。远端内容按设备分别存放；原本地开发仓库继续保留。2026-09-08 复制，实际版本以本次清单为准。','',
        '**优先参考：** [用户指出的现有可运行 Fixed Planner](FIXED_PLANNER_REFERENCE_2026-09-08.md)。Yichao Planner 仍在 sim2real 适配阶段，不能把现有 Fixed Planner 的运行经验记成 Yichao 已验收。','',
        '本次远程操作只有 SSH 读取、文件下载、目录/版本盘点和哈希计算；没有写入远程业务文件，没有启动或停止机器人控制程序。', '',
        '## 路径与范围','',
        '映射统一为 `<本地设备目录>/<远端绝对路径去掉开头斜线>`。例如 `odl@172.16.3.126/home/odl/codebase/pingpang_doubles_v9_runtime/` 对应工作站 `/home/odl/codebase/pingpang_doubles_v9_runtime/`。','',
        '本开发机保留原有 `pingpang_planner/`、`pingpang_controller/`、`pingpang_deploy/`、`yichao_v3_v9/` 和资产/文档。没有把个人工作站 `172.16.3.64` 列入此次三台远端复制范围。','']
    for key,(host,folder,_) in HOSTS.items():
        manifest=json.loads((ROOT/'doc/device_context'/f'{key}_manifest.json').read_text())
        body=[f'# {LABELS[key]} 本地副本','',f'来源：`{host}`。采集时间：`{manifest["captured_utc"]}`。', '',
              '本目录下 `home/...` 按远端原绝对路径布局。不同设备的内容分别保留。', '',
              '| 远端路径 / 本地入口 | HEAD（如有） | 常规文件体积 |', '| --- | --- | --- |']
        lines += [f'## {LABELS[key]}','',f'[设备目录与链接索引](../{folder}/README_CONTEXT.md)。','',
                  '| 本地目录 | 对应远端 | Git HEAD |','| --- | --- | --- |']
        for entry in manifest['roots']:
            remote=entry['remote'];git=entry['git']
            if entry.get('selected_files'):remote=str(Path(remote)/entry['selected_files'][0])
            rel=remote.lstrip('/')
            suffix='' if entry.get('selected_files') else '/'
            head=git['head']['stdout'].strip()[:12] if git and git['head']['code']==0 else '无独立 Git HEAD'
            size=sum(x.get('size',0) for x in entry['files'])/2**20
            body.append(f'| [{remote}]({rel}{suffix}) | `{head}` | {size:.1f} MiB |')
            lines.append(f'| [{remote.split("/")[-1]}](../{folder}/{rel}{suffix}) | `{remote}` | `{head}` |')
        body += ['', '## 符号链接','',
                 '链接文本原样保留。远端绝对链接在本机可能显示失效；请用下表的本地目标入口访问。本地没有改写这些链接，以便准确对照远端。', '',
                 '| 链接文件 | 原目标 | 本地目标 |','| --- | --- | --- |']
        for entry in manifest['roots']:
            for item in entry['files']:
                if item['kind']!='symlink':continue
                path=Path(entry['remote'])/item['path'];target=item['target']
                target_path=Path(target) if target.startswith('/') else path.parent/target
                local=target_path.as_posix().lstrip('/')
                body.append(f'| `{path}` | `{target}` | [本地入口]({local}) |')
        body += ['', '## 环境、版本与一致性','',
                 f'文件、目录、排除项、Git 工作树状态及依赖版本：[manifest](../doc/device_context/{key}_manifest.json)。',
                 f'逐文件远端 SHA256：[基线](../doc/device_context/{key}_sha256.json)。',
                 '[全局验证结果](../doc/device_context/verification.json)。[操作规则](../AGENTS.md)。', '',
                 'SDK、动作库、模型、配置、Git 历史和有用的交付/验证输出保留。未复制整个系统或可直接运行的环境；本机与机载架构可能不同。','']
        (ROOT/folder/'README_CONTEXT.md').write_text('\n'.join(body))
        (ROOT/folder/'AGENTS.md').write_text(f'''# {LABELS[key]} 设备副本

远端：`{host}`。本地 `home/...` 映射远端 `/home/...`。详见 [目录、版本与符号链接索引](README_CONTEXT.md)。

遵守项目根 [AGENTS.md](../AGENTS.md) 的同步、隔离和运行授权规则。已授权代码修改须同步对应远端并验证；他人现场基线仍作为参考保留。不要把本目录内模型、运行配置或版本混同另一台设备，也不要在本机执行复制来的启动或安装脚本。

本目录的 `AGENTS.md` 和 `README_CONTEXT.md` 是本地生成的导航文件，不属于远端待同步业务目录。SSH 密码只从项目根 AGENTS.md 读取，不复制到本目录或日志。
''')
        lines += ['', f'文件与排除项：[清单](device_context/{key}_manifest.json)；校验：[SHA256](device_context/{key}_sha256.json)。','']
    lines += ['## 明确省略的内容','',
        '- `.venv/`、`venv/`、`node_modules/` 和 Python/工具缓存；环境中的包名称和版本已读入设备 manifest，不安装或修改远端环境。',
        '- 各选定项目根的 `logs/`、`g1_gym_deploy/logs/`、`g1_gym_deploy/scripts/logs/`；Runtime 的 `outputs/ros_web_monitor_logs/`；CycloneDDS 工作区的 `log/`，以及机载临时运行锁 `onboard_shadow.lock`。',
        '- 仅按上述准确路径省略运行日志，SDK 中 `include/unitree/common/log/` 的源码、handoff 内训练交付、Yichao 的 `output/` 验证证据均保留。',
        '- 未选择的历史项目、设备操作系统、用户私人目录和账户配置不属于此次复制范围。','',
        '两个 `doubles-v10-i25000-dual-runtime/data/` 是 V11 副本绝对链接依赖而补充下载的动作数据，不表示开始适配 V10。每个 manifest 的 `excluded` 列表记录实际跳过的路径。工作站追加核查已覆盖整个 `codebase` 中的项目及相关顶层文件，并补充 `backups`、`staging`、桌面启动脚本和下载目录的相关数据/交付包。核查范围及明确不复制项见 [工作站补漏记录](WORKSTATION_CONTEXT_AUDIT_2026-09-08.md)。完整规则见 [复制配置](../tools/device_context_config.json)。','',
        '## 校验与维护','']
    if verification:
        state='通过' if verification['passed'] else '进行中，尚未全量通过'
        lines += [f'逐文件 SHA256 / 类型 / 权限 / 链接验证：**{state}**。时间：`{verification["captured_utc"]}`。', '',
                  '| 设备 | 范围目录/单文件 | 已验证常规文件 | 字节数 | 错误 / 缺失目录 |','| --- | --- | --- | --- | --- |']
        for k,d in verification['devices'].items():
            lines.append(f'| {k} | {d["roots"]} | {d["regular_files_checked"]} | {d["regular_bytes"]} | {len(d["errors"])} / {len(d["missing_directories"])} |')
    lines += ['', '[本地 SHA256 验证明细](device_context/verification.json)；[逐路径最新远端 checksum 验证汇总](device_context/remote_verification_summary.json)。下载和当前远端 rsync 校验记录位于 `doc/device_context/pull_*`、`dependencies_*`、`retry_*` 和 `verify_*`。文件校验不等于真机运行验收。','',
        '文件内容、符号链接文本和权限在选定范围内对照；文件所有者/组使用本机账户，不复制远端 UID/GID；目录 mtime 不作为内容一致性判据。绝对链接按设备 README 中映射访问。', '',
        '维护命令（在项目根执行）：','', '```bash',
        'python3 tools/device_context_verify.py',
        'python3 tools/device_context_pull.py --verify',
        '```','',
        '前者校验本地与已保存的 SHA256 基线；后者只读比较当前远端，发现变化返回失败并留下差异日志。初次下载或补缺可用 `python3 tools/device_context_pull.py`，该命令保护已存在的本地文件，不负责覆盖旧版本或自动解决冲突。更新必须先核对双方差异。', '',
        '网络传输中，对本机已存在且 SHA256 与远端完全相同的文件使用独立副本补齐，减少重复传输；没有把两台设备的可编辑文件建立共享硬链接。证据见 `seed_result.json`。', '',
        '后续同步是 Agent 的任务要求，没有设置后台监听/定时推送。修改前核对远端漂移，完成后按已授权文件清单备份、上传、哈希验证；详细约束以项目根 AGENTS.md 为准。','']
    (ROOT/'doc/DEVICE_CONTEXT_INDEX.md').write_text('\n'.join(lines))


if __name__=='__main__':main()
