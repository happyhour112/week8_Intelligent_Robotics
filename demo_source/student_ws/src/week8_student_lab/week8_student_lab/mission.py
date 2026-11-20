"""Hardware-free educational mission logic; no ROS imports here.

Implements: S -> find toy -> pickup -> find box -> deliver -> repeat.
For 'explore' mode, temporary waypoint goals are selected from observed free
frontiers, not from arbitrary unobserved/blocked locations.
"""
from math import hypot
from .world import START, TOYS, BOX, WALLS, W, H, inside, visible, reveal
from .search import astar, choose_frontier


class MissionModel:
    def __init__(self,mode='mission'):
        assert mode in ('mission','explore')
        self.mode=mode
        self.position=START
        self.carrying=None
        self.remaining=set(TOYS)
        self.delivered=0
        self.seen_toys=set(TOYS) if mode=='mission' else set()
        self.box_seen=mode=='mission'
        self.known=({(x,y):(100 if (x,y) in WALLS else 0)
                     for y in range(H) for x in range(W)} if mode=='mission' else {})
        self.phase='SEARCH_TOY'
        self.goal=None
        self.route=[]
        self.target_kind=''
        self.ticks=0
        self.excluded=set()
        self.last_event='Initialized'
        self._observe()

    def _observe(self):
        if self.mode != 'explore':
            return
        changed=reveal(self.known,self.position)
        if changed:
            self.excluded.clear()
        for toy in self.remaining:
            if (toy not in self.seen_toys and hypot(toy[0]-self.position[0],toy[1]-self.position[1])<=3.25
                    and visible(self.position,toy)):
                self.seen_toys.add(toy)
                self.last_event=f'Toy DETECTED at {toy} (mock vision)'
                if not self.carrying and self.target_kind=='frontier':
                    self.goal=None;self.route=[]
        if not self.box_seen and hypot(BOX[0]-self.position[0],BOX[1]-self.position[1])<=3.25 and visible(self.position,BOX):
            self.box_seen=True
            self.last_event='Toy box DETECTED (mock vision)'
            if self.carrying is not None and self.target_kind=='frontier':
                self.goal=None;self.route=[]

    def _plan(self):
        allowed=None if self.mode=='mission' else {p for p,value in self.known.items() if value==0}
        target=None;kind=''
        if self.carrying is not None and self.box_seen:
            target,kind=BOX,'box'
        elif self.carrying is None and self.seen_toys & self.remaining:
            choices=[]
            for toy in self.seen_toys & self.remaining:
                p,_=astar(self.position,toy,allowed=allowed)
                if p:
                    choices.append((len(p)-1,toy))
            if choices:
                choices.sort();target,kind=choices[0][1],'toy'
        if target is None:
            if self.mode!='explore':
                self.phase='FAILED'
                self.last_event='Target unavailable in fixed-map exercise'
                return
            target,route=choose_frontier(self.position,self.known,self.excluded)
            kind='frontier'
            if target is None:
                self.phase='FAILED'
                self.last_event='No reachable unexplored frontier; target not found'
                return
        else:
            route,_=astar(self.position,target,allowed=allowed)
        if not route:
            self.phase='FAILED';self.last_event=f'No route to {target}';return
        self.goal=target;self.target_kind=kind;self.route=route
        self.phase='EXPLORE' if kind=='frontier' else ('TO_BOX' if kind=='box' else 'TO_TOY')
        self.last_event=f'New {kind.upper()} goal {target}; route {len(route)-1} steps'

    def step(self):
        """One *discrete* robot movement step for an educational animation.

        No real /cmd_vel or motor physics are simulated.
        """
        if self.phase in ('DONE','FAILED'):
            return self.last_event
        self.ticks+=1
        self._observe()
        if self.carrying is None and self.position in self.remaining:
            self.carrying=self.position
            self.remaining.remove(self.position)
            self.seen_toys.discard(self.position)
            self.goal=None;self.route=[];self.target_kind=''
            self.phase='SEARCH_BOX'
            self.last_event=f'PICKED UP toy at {self.position} (mock grasp)'
            return self.last_event
        if self.carrying is not None and self.position==BOX:
            self.delivered+=1;self.carrying=None
            self.goal=None;self.route=[];self.target_kind=''
            self.phase='SEARCH_TOY' if self.remaining else 'DONE'
            self.last_event=f'DELIVERED toy {self.delivered} to box; {len(self.remaining)} remaining'
            return self.last_event
        if self.goal is not None and self.position==self.goal:
            if self.target_kind=='frontier':
                self.excluded.add(self.goal)
            self.goal=None;self.route=[]
        if not self.route:
            self._plan()
        if self.phase in ('DONE','FAILED'):
            return self.last_event
        if len(self.route)>1:
            nxt=self.route[1]
            if nxt in WALLS or (self.mode=='explore' and self.known.get(nxt)!=0):
                self.phase='FAILED';self.last_event='Unsafe step blocked';return self.last_event
            self.position=nxt
            self.route=self.route[1:]
            if self.ticks%10==0:
                self.last_event=f'Moving toward {self.target_kind} {self.goal}: pose {self.position}'
        return self.last_event
