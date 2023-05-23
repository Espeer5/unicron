"""
This module contains code that I wrote here because I was tired of writing in 
behaviors.
"""

import mapping.planning as pln


def master(exp, nav, stepping, step, sv, show, pickle=False):
    # Initialize hardware
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")

    # Instantiate hardware objects
    driveSys = DriveSystem(io, const.L_MOTOR_PINS, \
                                const.R_MOTOR_PINS, \
                                const.PWM_FREQ)
    sensor = LineSensor(io, const.IR_PINS)
    while((exp, nav) == (0, 0)):
        continue
    graph = None 
    if pickle:
        graph = pln.from_pickle()
    if graph == None and nav:
        raise Exception("Norman needs a map to navigate >:(")
    if exp and nav:
        raise Exception("Norman cannot explore and navigate at the same time >:(")
    location = (0, 0)
    heading = 0 
    prev_loc = (0, 0)
    tool = None
    if graph != None:
        tool = Visualizer(graph)
    djik = None 
    if graph != None:
        djik = Djikstra(graph, (0, 0))
    path = []
    try:
        while True:
            print(flags)
            if sv:
                complete(graph, tool)
                sv = False
            if show:
                if tool = None:
                    print("Norman has no map to display >:(")
                else:
                    tool.show()
            if act.line_follow(driveSys, sensor) == const.SUCCESS:
                if steppping:
                    while not step:
                        continue
                time.sleep(.2)
                prev_loc = location
                location = (location[0] + const.heading_map[heading][0], 
                            location[1] + const.heading_map[heading][1])
                if graph == None:
                        graph, tool, djik = pln.init_plan(location, heading)
                else:
                    graph.driven_connection(prev_loc, location, heading)
                if nav:
                    if path != []:
                    direction = act.to_head(heading, path[0], graph, location)
                    while heading != path[0]:
                        angle = abs(act.exec_turn(driveSys, sensor, direction))
                        heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
                    time.sleep(.02)
                    continue
                    else:
                        cmd = input("Enter new coordinates to drive to: ")
                        dest = (int(cmd.split(",")[0]), int(cmd.split(",")[1]))
                        while not graph.contains(dest):
                            print("Location does not exist!")
                            dest = input("Enter valid coordinates")
                        print("Driving to (" + str(dest[0]) + ", " + str(dest[1]) + ")...")
                        # run algorithm and determine best path to travel
                        djik.reset(dest)
                        path = djik.gen_path(location)
                        direction = act.to_head(heading, path[0], graph, location)
                        while heading != path[0]:
                            angle = abs(act.exec_turn(driveSys, sensor, direction))
                            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
                        time.sleep(.02)
                        break
                if exp:
                    check_end(sensor, graph, location, heading)
                    if path != []:
                        direction = act.to_head(heading, path[0], graph, location)
                        while heading != path[0]:
                            angle = abs(act.explore_turn(driveSys, sensor, direction))
                            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
                        time.sleep(.02)
                        continue
                    graph, location, heading, explored = auto_inters(driveSys, sensor, graph, heading, location)
                    if explored:
                        continue
                    else:
                        #Otherwise, use Djikstra to find an efficient path to an unexplored location
                        dest = pln.find_unexplored(graph, location)
                        if dest == None:
                            direc = pln.unx_dir(graph.get_intersection(location))
                            if direc == None:
                                direc = heading
                            heading = explore_turn(driveSys, sensor, act.to_head(heading, direc, graph, location), graph, location, heading)
                            break
                        djik.reset(dest)
                        path = djik.gen_path(location)
                        direction = act.to_head(heading, path[0], graph, location)
                        while heading != path[0]:
                            angle = abs(act.exec_turn(driveSys, sensor, direction))
                            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
                        time.sleep(.02)
                        break
    except KeyboardInterrupt:
        driveSys.stop()
        io.stop()

