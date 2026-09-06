#!/usr/bin/env python3
"""Production-law closed-loop regression; simplified skid kinematics, not Gazebo."""
import csv
import io
import math
import subprocess
import sys
import tempfile
from pathlib import Path

replay = sys.argv[1]
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'reference.csv'
    for speed, offset, yaw in [(0.3, .229, 0), (-.3, -.229, 0), (.3, .229, .05)]:
        path.write_text(''.join(f'{i*.1},{speed*i*.1},0,{speed},0,0,0\n' for i in range(201)))
        output = subprocess.check_output([replay, str(path), '6', '4', '.005', '.010', '.5235', str(offset), '.01', str(yaw)], text=True)
        rows = [{k: float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(output))]
        assert all(math.isfinite(v) for row in rows for v in row.values())
        assert max(row['error'] for row in rows) < .1, (speed, offset, yaw, 'unbounded error')
        assert rows[-1]['error'] < .01, (speed, offset, yaw, 'did not converge')
        assert max(abs(row['cmd_w']) for row in rows) < .3, (speed, offset, yaw, 'excess yaw command')
    # Continuous position with a stop and reversal: the law must neither spin
    # at rest nor keep moving in the original direction after the command reverses.
    x=0.0
    samples=[]
    for i in range(201):
        t=i*.1
        v=.3 if t<8 else (0.0 if t<12 else -.3)
        samples.append(f'{t},{x},0,{v},0,0,0\n')
        x+=v*.1
    path.write_text(''.join(samples))
    output=subprocess.check_output([replay,str(path),'6','4','.005','.010','.5235','.229','.01'],text=True)
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(output))]
    assert max(row['error'] for row in rows)<.12
    assert rows[-1]['body_v']<-.25
    assert max(abs(row['body_w']) for row in rows if 11<row['t']<12)<.01
print('production flatness skid-response regressions passed')
