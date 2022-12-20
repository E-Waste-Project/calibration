data = []
with open('rob.txt') as f:
  for line in f.readlines():
    a_line = line.strip().split(',')
    a_line = list(map(lambda x:float(x), a_line))
    print(a_line)