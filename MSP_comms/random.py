list1 = []
list2= [1]
while len(list1)<len(list2):
    list1 = list2.copy()
    list2.append(1/3)
    print(len(list2))

