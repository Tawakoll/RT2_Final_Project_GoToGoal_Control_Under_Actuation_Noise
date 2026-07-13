#!/bin/bash
# Smoke test: start the experiment node, request one trial per controller,
# verify that results come back and are logged to CSV.
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

cd /root
rm -f /root/smoke_results.csv /root/node.log

ros2 run goal_nav_experiment experiment_node --ros-args -p csv_path:=/root/smoke_results.csv > /root/node.log 2>&1 &
NODE_PID=$!
sleep 4

python3 - <<'EOF'
import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('smoke_client')
results = {}
sub = node.create_subscription(
    String, '/experiment/result',
    lambda m: (lambda r: results.setdefault(r['trial_id'], r))(json.loads(m.data)),
    10)
pub = node.create_publisher(String, '/experiment/run', 10)
time.sleep(1.0)

for i, (ctrl, noise) in enumerate([('proportional', 0.0),
                                   ('rotate_translate', 0.0),
                                   ('proportional', 0.3)]):
    req = {'controller': ctrl, 'noise_std': noise, 'seed': 7, 'trial_id': i}
    pub.publish(String(data=json.dumps(req)))
    t0 = time.time()
    while i not in results and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)

print(f'received {len(results)} results')
for r in results.values():
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
assert len(results) == 3, 'expected 3 results'
assert results[0]['success'] and results[1]['success'], 'noise-free trials must succeed'
assert results[0]['time_to_goal'] < results[1]['time_to_goal'], \
    'proportional expected faster than rotate_translate on same seed'
assert all(r['efficiency'] <= 1.0 for r in results.values()), 'efficiency must be <= 1'
print('SMOKE TEST PASSED')
node.destroy_node()
rclpy.shutdown()
EOF
STATUS=$?

kill $NODE_PID 2>/dev/null
echo "--- csv ---"
cat /root/smoke_results.csv 2>/dev/null
echo "--- node log (tail) ---"
tail -5 /root/node.log
exit $STATUS
