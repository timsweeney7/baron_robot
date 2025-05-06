from utilities.path_planner import WorldMap
from utilities.block import Block

block_1 = Block(
    color="red",
    location=(1.3, 1.3),
)
block_2 = Block(
    color="cyan",
    location=(1.55,1.3),
)
block_3 = Block(
    color="green",
    location=(1.6, 1.8),
)
# block_4 = Block(
#     color="cyan",
#     location=(0.2, 0.2),
# )

    

MAP_LENGTH = 3.048 # 10ft
MAP_WIDTH = 2.7432 # 9ft

wm = WorldMap(MAP_LENGTH, 
              MAP_WIDTH, 
              None
              )
wm.blocks = [block_3, block_2]
wm.robot_position = (1, 1)
wm._debug_heading = 30
wm.draw_map()


input("puase...")

pos, head = wm.get_robot_position()
meta_data = {
    "position": pos,
    "heading_deg": head,
}
print(meta_data)


block_2 = Block(
    color="cyan",
    location=(1.6,1.3),
)

new_blocks = [block_2, block_1]

wm.update_blocks(new_blocks, meta_data)
wm.draw_map()


