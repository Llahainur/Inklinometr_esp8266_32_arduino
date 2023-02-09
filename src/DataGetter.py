import requests

url = 'http://192.168.1.1'
html = requests.get(url).text

with open('test.html', 'w') as test_file:
    page = bytes(html, 'utf-8')
    test_file.write(str(page))