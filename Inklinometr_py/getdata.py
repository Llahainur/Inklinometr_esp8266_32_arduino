import requests
import time


url = 'http://192.168.1.1'
html = requests.get(url).text
log=""
x=[]
y=[]
z=[]
num=[]

for i in range(0,60):
    html = requests.get(url).text
    arr=html.split(' ')
    x.append(arr[0])
    y.append(arr[2])
    z.append(arr[4])
    num.append(arr[6])
    print(i)
    print(x,y,z, num)
    time.sleep(60*1)

with open('log.txt', 'w') as test_file:
    for i in range(len(x)):
        log += str(x[i])+" "+str(y[i])+" "+str(z[i])+" "+str(num[i])
    test_file.write(log)