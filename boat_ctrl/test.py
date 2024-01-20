import json

test = []
test.append(
    {
        "a": 1,
        "b": 2
    }
)


a = json.dumps(test)

b = json.loads(a)
print(type(b))
print(b)