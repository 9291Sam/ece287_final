import random;


for i in range(255):
    print(f"rand[{random.randrange(255)}] ^ ~rand[{random.randrange(255)}] ^ rand[{random.randrange(255)}],");