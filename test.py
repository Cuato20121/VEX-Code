unordereddistanceofirregularities = []
test = []
for i in range(10):
    unordereddistanceofirregularities.append({'turn': i, 'distance': i})


for i, a in enumerate(unordereddistanceofirregularities):
    tempdict1 = unordereddistanceofirregularities[i]
    test.append(tempdict1['distance'])

for i in test:
    print(i)
