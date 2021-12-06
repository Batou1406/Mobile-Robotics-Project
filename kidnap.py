from tdmclient import ClientAsync, aw

client = ClientAsync()
node = aw(client.wait_for_node())

aw(node.lock())
aw(node.wait_for_variables())

dist_threshold = 50
# sensor_vals = list(node['prox.ground.reflected'])

async def is_kidnap():
    print(list(node['prox.ground.reflected']))
    if all([x < dist_threshold for x in node['prox.ground.reflected']]):
        print("Kidnap")

async def run():
    while True:
        await is_kidnap()

aw(run())
aw(node.unlock())