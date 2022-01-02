def log(msg):
    print(msg)
    with open("log.txt", 'a') as f:
        f.write("\n" + str(msg))
