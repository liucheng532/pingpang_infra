# V7 deterministic replay fixture

`v7_two_ball.jsonl` contains two distinct synthetic shots and one duplicate event
for each shot. It is local test input, not real robot telemetry. The replay tool
uses it to prove one inference per `shot_id`, latched duplicate-shot results,
29D/87D ABI handling, and a zero-publisher offline path.
