from Constants import REWARDS

exps = set()
seen = set()

for beg in range(6):
    beg_r = REWARDS[beg]
    for end in range(1,7):
        if beg == end: continue
        end_r = REWARDS[end]
        exp = (beg,end)
        try1 = (beg_r[0], beg_r[1], end_r[0], end_r[1])
        try2 = (beg_r[1], beg_r[0], end_r[1], end_r[0])
        try3 = (beg_r[0], beg_r[1], end_r[1], end_r[0])
        if try1 not in seen:
            seen.add(try1)
            seen.add(try2)
            seen.add(try3)
            exps.add(exp)
            print "Adding ", exp, try1
        else:
            print "skipping ", exp, try2

print "experiments=", len(exps)
xp = list(exps)
xp.sort()
print xp


