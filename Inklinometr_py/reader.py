from statistics import *


v=[]
with open('log.txt', 'r') as f:
    txt=f.read()
    strings = txt.split(';')
    for val in strings:
        v.append(val.split(' '))
x=[]
y=[]
z=[]


for line in v:
    for i in range(len(line)):
        # print(line[i])
        if line[i] == '': continue
        if i==0:
            x.append(float(line[i]))
        elif i==1:
            y.append(float(line[i]))
        elif i==2:
            z.append(float(line[i]))
print("X", mean(x), pstdev(x))
print("Y", mean(y), pstdev(y))
print("Z", mean(z), pstdev(z))








