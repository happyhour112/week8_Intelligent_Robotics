"""Small, tested A* on the precise teaching maze (4-neighbour grid)."""
import heapq
from .world import inside, neighbors, WALLS, W, H


def astar(start, goal, blocked=None, allowed=None):
    """Return (path as cells including endpoints, nodes expanded).

    allowed: set of observed-free cells for frontier mode, None for known map.
    blocked: set of impassable cells, default full wall list.
    Movement cost: 1 per 4-connected step; h: Manhattan distance.
    """
    blocked = WALLS if blocked is None else blocked
    def free(c):
        return inside(c) and c not in blocked and (allowed is None or c in allowed)
    if not free(start) or not free(goal):
        return [], 0
    h=lambda c: abs(goal[0]-c[0])+abs(goal[1]-c[1])
    g={start:0}
    parent={}
    openq=[(h(start),0,start)]
    expanded=0
    closed=set()
    while openq:
        _,gc,node=heapq.heappop(openq)
        if node in closed or gc!=g[node]:
            continue
        expanded+=1
        if node==goal:
            result=[node]
            while node in parent:
                node=parent[node];result.append(node)
            return list(reversed(result)), expanded
        closed.add(node)
        for nxt in neighbors(node):
            if not free(nxt) or nxt in closed:
                continue
            alt=g[node]+1
            if alt<g.get(nxt,float('inf')):
                g[nxt]=alt;parent[nxt]=node
                heapq.heappush(openq,(alt+h(nxt),alt,nxt))
    return [],expanded


def frontiers(known):
    """Observed free cells touching at least one unknown cell."""
    return [c for c,v in known.items() if v==0 and any(inside(n) and n not in known for n in neighbors(c))]


def choose_frontier(robot,known,excluded=frozenset()):
    """Nearest reachable frontier with deterministic tie-break.
    Exclude current cell to avoid an endless no-movement loop.
    """
    free={p for p,v in known.items() if v==0}
    candidates=[]
    for f in frontiers(known):
        if f==robot or f in excluded:
            continue
        p,_=astar(robot,f,allowed=free)
        if p:
            candidates.append((len(p)-1,-f[0],f[1],f,p))
    if not candidates:
        return None,[]
    candidates.sort()
    return candidates[0][3],candidates[0][4]
