def get_json_from_grid_msg(grid_msg):
    return '{'\
        '"info": {'\
            '"resolution": ' + str(grid_msg.info.resolution) + ','\
            '"width": ' + str(grid_msg.info.width) + ','\
            '"height": ' + str(grid_msg.info.height) + ','\
            '"origin": {'\
                '"position": {'\
                    '"x": ' + str(grid_msg.info.origin.position.x) + ","\
                    '"y": ' + str(grid_msg.info.origin.position.y) + ","\
                    '"z": ' + str(grid_msg.info.origin.position.z) + ""\
              '},'\
                '"orientation": {'\
                    '"x": ' + str(grid_msg.info.origin.orientation.x) + ","\
                    '"y": ' + str(grid_msg.info.origin.orientation.y) + ","\
                    '"z": ' + str(grid_msg.info.origin.orientation.z) + ","\
                    '"w": ' + str(grid_msg.info.origin.orientation.w) + ""\
                '}'\
            '}'\
        '}'\
    '}'
