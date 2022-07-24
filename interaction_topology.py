


topology = [[0  for i in range(30)] for j in range(30)]

for i in range(5,30):
    if i==29 or i==9 or i==14 or i==19 or i==24:
        continue
    topology[i][i+1]=1
    topology[i+1][i]=1

adjacent = [ [1,5,29,5,17],[9,22,3,10,2],[14,0,27,15],[19,20,7],[25,24,12]]

for i in range(5):
    for j in adjacent[i]:
        topology[i][j] = 1

print(topology)
