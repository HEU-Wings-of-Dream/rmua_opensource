import os

abs_path = os.getcwd()

print (abs_path)

filenames=os.listdir('D:\\ProgramData\\outputs3')

listfile = open('./other_program/listfile.txt', 'r+')

for i in filenames:
    
    listfile.write('%s\n' %(i))

print ("save success")