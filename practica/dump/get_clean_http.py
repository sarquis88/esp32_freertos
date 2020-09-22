import json
from contextlib import suppress
  
i = 0

with open('dump.json') as f:
    data = json.load(f)

try:
    while True:
        print( data[i]['_source']['layers']['json']['json.object']['json.member']['json.value.string'] )
        i = i + 1
except IndexError as error:
    exit()


