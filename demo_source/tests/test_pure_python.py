"""Run from the lab root: python3 tests/test_pure_python.py (NO ROS needed)."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'student_ws'/'src'/'week8_student_lab'))
from week8_student_lab.world import START,TOYS,BOX,W,H,WALLS,center,visible,reveal
from week8_student_lab.search import astar,frontiers,choose_frontier

def run():
    for goal in (*TOYS,BOX):
        assert goal not in WALLS,(goal,'blocked')
        p,n=astar(START,goal)
        assert p and p[0]==START and p[-1]==goal
        for a,b in zip(p,p[1:]):
            assert abs(a[0]-b[0])+abs(a[1]-b[1])==1
    p,_=astar(START,BOX)
    assert len(p)-1==26,(len(p)-1,'expected 26')
    known={}
    assert reveal(known,START)>0
    assert 0<len(known)<W*H
    f,route=choose_frontier(START,known)
    assert f is not None and route and route[0]==START
    print('PASS: graph A*, known map, unknown-map scan and first frontier')
    # Mission with simulated exploration, no ROS needed
    from week8_student_lab.mission import MissionModel
    s=MissionModel('explore')
    actions=[]
    for i in range(2500):
        a=s.step();actions.append(a)
        if s.phase in ('DONE','FAILED'):
            break
    print('explore steps=',len(actions),'phase=',s.phase,'completed=',s.delivered,'known=',len(s.known))
    assert s.phase=='DONE',(s.phase,s.position,s.delivered)
    assert s.delivered==len(TOYS)
    # Also test static mission and bounds
    m=MissionModel('mission')
    for i in range(500):
        m.step()
        if m.phase in ('DONE','FAILED'):
            break
    assert m.phase=='DONE' and m.delivered==len(TOYS)
    print('PASS: full mission + frontier exploration and 2 toy deliveries')
if __name__=='__main__':run()
