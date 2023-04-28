import asyncio
import nats

from sense_hat import SenseHat
from math import pi, sin, cos

async def main():

    sense = SenseHat()

    nc = await nats.connect('nats://sml-wan.asuscomm.com:4222')

    while True:

        o = sense.get_orientation()
        pitch = o["pitch"]
        roll = o["roll"]
        forward = sin(pitch * pi / 180)
        forward = (forward+1) * 4
        forward = int(forward)
        right = -sin(roll * pi / 180)
        right = (right+1) * 4
        right = int(right)
        sense.clear()
        sense.set_pixel(7 - forward, 7 - right, (255, 255, 255)) #red, gren, blue
        msg = "pitch: {}, roll: {}, forward: {}, right: {}".format(pitch, roll, forward, right)

        await nc.publish("clientA_talker", msg.encode())

        await asyncio.sleep(0.1)

    await nc.close()

if __name__ == '__main__':
    asyncio.run(main())
