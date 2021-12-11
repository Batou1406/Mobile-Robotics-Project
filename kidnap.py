from tdmclient import ClientAsync, aw


class KidNap:
    def __init__(self):
        self.client = ClientAsync()
        self.node = aw(self.client.wait_for_node())

        aw(self.node.lock())
        aw(self.node.wait_for_variables())
        self.dist_threshold = 70
        self.sensor_vals = list(self.node['prox.ground.reflected'])

    async def is_kidnap(self):
        # print(list(self.node['prox.ground.reflected']))
        await self.client.wait_for_node()
        self.sensor_vals = list(self.node['prox.ground.reflected'])
        if all([x < self.dist_threshold for x in self.sensor_vals]):
            print("Kidnap")

    async def run(self):
        while True:
            print(self.sensor_vals)
            # print(self.node.var)
            # self.sensor_vals = list(self.node['prox.ground.reflected'])
            await self.is_kidnap()

kidnap = KidNap()
aw(kidnap.run())
# aw(node.unlock())