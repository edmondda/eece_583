import csv
import argparse
from graphics import *
import numpy as np

def read_info(reader) :

    (xdim, ydim) = next(reader)
    xdim = int(xdim)
    ydim = int(ydim)

    num_obstr = int(next(reader).pop())

    obstr_loc = []

    for obstr in range(num_obstr) :
        obstr_loc.append( tuple([ int(s) for s in next(reader) ]) )

    # read and plot pin pairs
    pin_groups = int(next(reader).pop())

    pin_locs = []

    for pin_group in range(pin_groups) :

        pin_list = next(reader)
        num_pin = int(pin_list.pop(0))
        pin_loc = []

        for pin in range(num_pin) :
            pin_loc.append( ( int(pin_list[(pin)*2]), int(pin_list[(pin)*2 + 1]) ) )

        pin_locs.append(pin_loc)

    return xdim, ydim, obstr_loc, pin_locs


def draw_example(ex_file, pxscale, xdim, ydim, obstr_loc, pin_locs, colors) :

    win = GraphWin(ex_file, xdim*pxscale , ydim*pxscale)

    # draw grid
    for x in range(xdim):
        for y in range(ydim):
            win.plotPixel(x*pxscale, y*pxscale, "black")

    for obstr in obstr_loc :
        rect = Rectangle( Point(pxscale*obstr[0], pxscale*obstr[1]), \
                          Point(pxscale*(obstr[0] + 1), pxscale*(obstr[1] + 1)) )
        rect.setOutline('black')
        rect.setFill('red')
        rect.draw(win)

    for pin_group, pin_no in zip( pin_locs, range(len(pin_locs)) ) :
        for pin in pin_group :
            rect = Rectangle( Point(pxscale*pin[0], pxscale*pin[1]), \
                              Point(pxscale*(pin[0] + 1), pxscale*(pin[1] + 1)) )
            rect.setOutline('black')
            rect.setWidth(2)
            rect.setFill(colors[pin_no%12])
            rect.draw(win)

    return win


def draw_path(win, pxscale, path, pin_loc, colors) :

    for net_group, pin_group, path_no in zip( path, pin_loc, range(len(path)) ) :
        for net in net_group :
            rect = Rectangle( Point(pxscale*net[0], pxscale*net[1]), \
                              Point(pxscale*(net[0] + 1), pxscale*(net[1] + 1)) )
            if (net in pin_group) :
                rect.setOutline('black')
                rect.setWidth(2)
            rect.setFill(colors[path_no%12])
            rect.draw(win)



def draw_arrow(win, pxscale, loc, direction ) :

    if   (direction == 1) : #left
        (x1, y1) = ( (pxscale*(1 + loc[0])) , pxscale*loc[1] + pxscale/2 )
        (x2, y2) = ( (pxscale*loc[0]) , pxscale*loc[1] + pxscale/2 )
    elif (direction == 2) : #right
        (x1, y1) = ( (pxscale*loc[0]) , pxscale*loc[1] + pxscale/2 )
        (x2, y2) = ( (pxscale*(1 + loc[0])) , pxscale*loc[1] + pxscale/2 )
    elif (direction == 3) : #down
        (x1, y1) = ( pxscale*loc[0] + pxscale/2 , (pxscale * loc[1])  )
        (x2, y2) = ( pxscale*loc[0] + pxscale/2 , (pxscale*( 1 + loc[1])) )
    else : #up
        (x1, y1) = ( pxscale*loc[0] + pxscale/2 , (pxscale*(1 + loc[1])) )
        (x2, y2) = ( pxscale*loc[0] + pxscale/2 , (pxscale*loc[1]) )

    line = Line( Point(x1, y1), Point(x2, y2) )

    line.setArrow('last')
    line.setWidth(3)
    line.draw(win)

# sort according to min manhattan distance from source pin to destination pins
def sort_min_manhattan(src_net, dest_pins) :

    cnt = []
    i = 0

    for src_n in src_net :
        cnt_temp = []
        for dest_pin in dest_pins :
            #cnt_temp += abs(dest_pin[0] - src_n[0]) + abs(dest_pin[1] - src_n[1])
            cnt_temp.append(  ( abs(dest_pin[0] - src_n[0]) + abs(dest_pin[1] - src_n[1] ) ) )

        cnt.append(min(cnt_temp))

    src_net_temp = []
    src_net_temp = [ src_net[i] for i in np.argsort(cnt) ]
    return src_net_temp


def trace_path(s, c, dest_pin, traceback) :

    path_list = []
    path_list.append( dest_pin )

    src_found = False
    count = 0
    trace_fail = False
    while (not src_found) :

        exp_pt = path_list[-1]
        temp_loc = (  exp_pt[0] , exp_pt[1] )

        if traceback == 1 : # left
            nxt_loc = ( temp_loc[0] + 1, temp_loc[1] )
        elif traceback == 2 : # right
            nxt_loc = ( temp_loc[0] - 1, temp_loc[1] )
        elif traceback == 3 : # down
            nxt_loc = ( temp_loc[0], temp_loc[1] - 1 )
        elif traceback == 4 : # up
            nxt_loc = ( temp_loc[0], temp_loc[1] + 1 )

        traceback = s[ nxt_loc[0], nxt_loc[1] ]

        if traceback < 5 :
            path_list.append( nxt_loc )

        if traceback == 5 :
            src_found = True;

        count = count + 1
        if count > 100000 :
            print("traceback failed")
            trace_fail = True
            break


    return trace_fail, path_list


def draw_dot(win, pxscale, loc ) :

    (x1, y1) = ( pxscale*loc[0] + pxscale/2 , pxscale*loc[1] + pxscale/2 )

    circle = Circle( Point(x1, y1), 2 )

    circle.setFill('black')
    circle.draw(win)

# determine if a move from one location to another is closer to a group of destination pins
def torward_target(route, temp_loc, dest_pins) :

    toward_target = False

    for dest_pin in dest_pins :

        before_dist = abs(dest_pin[0] - route[0]) + abs(dest_pin[1] - route[1])
        neigh_dist  = abs(dest_pin[0] - temp_loc[0]) + abs(dest_pin[1] - temp_loc[1])

        if neigh_dist < before_dist :
            toward_target = True


    return toward_target


def toward_closest_target(route, temp_loc, dest_pins) :

    dest_pins_i = sort_min_manhattan(dest_pins, [route])

    toward_target = False

    dest_pin = dest_pins_i[0]
    #print("dset pin")
    #print(dest_pin)
    before_dist = abs(dest_pin[0] - route[0]) + abs(dest_pin[1] - route[1])
    neigh_dist  = abs(dest_pin[0] - temp_loc[0]) + abs(dest_pin[1] - temp_loc[1])

    if neigh_dist < before_dist :
        toward_target = True

    return toward_target


def reorder_nets(pin_locs) :

    thres_list = []
    min_list = []

    for pin_group in pin_locs :
        # find min/max x and y box
        x_max, y_max = pin_group[0]
        x_min, y_min = pin_group[0]
        for pin in pin_group :
            if pin[0] > x_max :
                x_max = pin[0]
            elif pin[0] < x_min :
                x_min = pin[0]
            if pin[1] > y_max :
                y_max = pin[1]
            elif pin[1] < y_min :
                y_min = pin[1]

        thres_list.append( [ x_max, y_max, x_min, y_min] )

    bbcnt_list = []
    for thres_vals, idx in zip( thres_list, range(len(thres_list)) ) :

            # determine how many pins are in the bounding box for
            # iterate over the pin list

            pin_idx = 0
            boundcnt = 0
            for pin_group in pin_locs :

                if (pin_idx != idx) :
                    for pin in pin_group :

                        if ( pin[0] <= thres_vals[0] and \
                             pin[1] <= thres_vals[1] and \
                             pin[0] >= thres_vals[2] and \
                             pin[1] >= thres_vals[3] ) :
                           boundcnt = boundcnt + 1

                pin_idx = pin_idx + 1

            bbcnt_list.append( boundcnt )


    #print(pin_locs)
    #print(bbcnt_list)

    # sort the pin lists in descending order of the number of pins in their bb
    src_net_temp = [ pin_locs[i] for i in np.argsort(bbcnt_list) ]


    return src_net_temp


## Lee-Moore
def leemoore(win, xdim, ydim, src_net, dest_pins, pin_locs, obstr_loc, pxscale, step_iter, net_iter ) :

    if (net_iter) :
        for src_pin in src_net :
            label = Text( Point( src_pin[0]*pxscale + pxscale/2, src_pin[1]*pxscale + pxscale/2), '1')
            label.draw(win)

    exp_list = []
    exp_list.append( src_net )
    obstr_loc_temp = src_net + obstr_loc

    #DE - Add other groups of pins to obstructions
    obstr_pins = []
    for pin_group in pin_locs :
        obstr_pins += pin_group

    # expand around
    target_found = False;
    no_solution = False;
    exp_cnt = 0;

    while (not no_solution and not target_found) :

        tmp_exp_list = []

        for exp_pt in exp_list[exp_cnt] :

            # exapand around this in all directions
            for (offset_x, offset_y) in [[1, 0], [0,1], [-1,0], [0,-1]] :

                # DE - consider co=ordinate class here
                temp_loc = ( ( exp_pt[0] + offset_x ), ( exp_pt[1] + offset_y ) )
                # Is this expanded point obstructed, or outside boundary
                if temp_loc not in obstr_loc_temp and \
                   temp_loc[0] >= 0 and temp_loc[0] < xdim and \
                   temp_loc[1] >= 0 and temp_loc[1] < ydim  and \
                   temp_loc not in obstr_pins:

                    tmp_exp_list.append(temp_loc)
                    obstr_loc_temp.append(temp_loc)
                    if (net_iter) :
                        label = Text( Point( temp_loc[0]*pxscale + pxscale/2, temp_loc[1]*pxscale + pxscale/2), exp_cnt+2)
                        label.draw(win)

        if tmp_exp_list :

            exp_list.append(tmp_exp_list)
            exp_cnt = exp_cnt + 1

            # are targets complete?
            reached_pins = list( set( dest_pins ).intersection( set( tmp_exp_list )) )
            target_found = len( reached_pins ) >= 1

            #  create path list
            if (target_found) :
                # just pick one pin if more than one is hit, it can be done in another iteration
                path_list = [ reached_pins[0] ]
                path_cnt = 0

                # remove last and first entry before selecting path
                for exp_pins in exp_list[1:-1][::-1] :
                    for exp_pin in exp_pins :
                        if sum( [abs(a_i - b_i) for a_i, b_i in zip(exp_pin, path_list[path_cnt] ) ] ) == 1 :
                            path_list.append(exp_pin)
                            path_cnt = path_cnt + 1;
                            break

                # remove the matched pin(s) from the dest_pins list
                dest_pins = list( set( dest_pins ).difference( set( [ reached_pins[0] ] ) ) )

                path_list = path_list + src_net

        else :

            no_solution = True

            # Pass on the path from prior iterations
            path_list = src_net


        if(step_iter) : win.getKey()

    return no_solution, dest_pins, path_list





## Line-probe
def lineprobe_mod(win, xdim, ydim, src_net, dest_pins, pin_locs, obstr_loc, pxscale, step_iter, net_iter ) :

    #for src_pin in src_net :
    #    label = Text( Point( src_pin[0]*pxscale + pxscale/2, src_pin[1]*pxscale + pxscale/2), '1')
    #    label.draw(win)


    #DE - Add other groups of pins to obstructions
    obstr_pins = []
    for pin_group in pin_locs :
        obstr_pins += pin_group

    # expand around
    target_found = False;
    no_solution = False;

    signal_def = np.zeros((xdim, ydim))
    reach_flag = np.zeros((xdim, ydim))

    rn = []
    ro = []

    # set source subnet = 5
    for src_net_i in src_net :
        signal_def[src_net_i[0], src_net_i[1]] = 5

    # set obstructions and other pins = 7
    for obstr_i in obstr_loc :
        signal_def[obstr_i[0], obstr_i[1]] = 7
    for obstr_i in obstr_pins :
        signal_def[obstr_i[0], obstr_i[1]] = 7


    # destination pins = 6
    for dest_pin in dest_pins :
        signal_def[dest_pin[0], dest_pin[1]] = 6

    # start closest manhatan distance from destination
    src_net_int = sort_min_manhattan(src_net, dest_pins)


    path_list = []
    path_list += src_net_int
    # add source net to rn
    rn = src_net_int
    src_pin = rn[0]


    # set the source pin to line-probe code
    reach_flag[src_pin[0], src_pin[1] ] = 2


    #for route in ro[::-1] :
    while (target_found == False and no_solution == False ) :

        # If we run out of RO, move RN to RO
        ro = rn
        rn = []
        ro.reverse()

        # loop through old route, RO
        while( ro ) :

               if (step_iter == True) :
                   win.getKey()

               # take last entry from old route
               ro_temp = ro.pop()
               cont_dir = False

               # add co-ordinates from last wave to next entry from RO
               #  clear the last wave
               ro_temp = [ ro_temp ] + rn
               rn = []

               # expand around last wave + next entry in RO
               while (ro_temp) :

                   route = ro_temp.pop()

                   # check signal definition and trace flag for neighbours
                   for (offset_x, offset_y, traceback) in [[1, 0,2], [0,1,3], [-1,0,1], [0,-1,4]] :

                       temp_loc = ( ( route[0] + offset_x ), ( route[1] + offset_y ) )

                       # determine if neighbour is toward the closest target in the
                       # dest_pins list
                       to_target = toward_closest_target(route, temp_loc, dest_pins )

                       if temp_loc[0] >= 0 and temp_loc[0] < xdim and \
                          temp_loc[1] >= 0 and temp_loc[1] < ydim  :
                          # signal definition and reach flag look-up for current location

                          s = signal_def[temp_loc[0], temp_loc[1] ]
                          c = reach_flag[temp_loc[0], temp_loc[1] ]

                          # Add expansion point if not and obstruction, previous line-probe, or source net
                          if not (c == 2 or s == 7 or s == 5) :
                              # target found
                              if s == 6 :

                                  if ( signal_def[ temp_loc[0], temp_loc[1] ] <= 4 ):
                                      signal_def[ temp_loc[0], temp_loc[1] ] = traceback
                                  target_found = True
                                  break;

                              # only line-probe if the point is not covered by previous expansion
                              elif c == 0 and to_target == True :

                                  # Add Rn to old route stack so that wave history isn't lost
                                  ro = ro + rn

                                  # Clear Rn
                                  rn = []

                                  # Iteratate towards for line-probe
                                  cont_dir = True
                                  while  (cont_dir == True) :

                                      # draw arrow with traceback direction
                                      if(net_iter) :
                                          draw_arrow(win, pxscale, temp_loc, traceback)

                                      # add line-probe point to RO
                                      #ro.insert(0, temp_loc)
                                      ro.append(temp_loc)

                                      # set the reach flag = 2
                                      reach_flag[ temp_loc[0], temp_loc[1] ] = 2

                                      if ( signal_def[ temp_loc[0], temp_loc[1] ] <= 4 ):

                                          # set traceback code
                                          signal_def[ temp_loc[0], temp_loc[1] ] = traceback

                                          # continue in the same line-probe direction
                                          temp_loc2 = ( ( temp_loc[0] + offset_x ), ( temp_loc[1] + offset_y ) )
                                          # Determine signal definition and reach flag of
                                          s = signal_def[ temp_loc2[0], temp_loc2[1] ]
                                          c = reach_flag[ temp_loc2[0], temp_loc2[1] ]
                                          cont_dir =     toward_closest_target(temp_loc, temp_loc2, dest_pins)
                                          temp_loc = temp_loc2
                                          if c == 2 or s == 7 or s == 5 and c != 1:
                                              # back to loop step 3
                                              #print("obstruction hit during probe")
                                              cont_dir = False
                                          elif s == 6 :
                                              # target found

                                              if ( signal_def[ temp_loc[0], temp_loc[1] ] <= 4 ):
                                                  signal_def[ temp_loc[0], temp_loc[1] ] = traceback
                                              target_found = True
                                              cont_dir = False


                                  # once line-probe is started, don't search other adjacent tiles
                                  # DE - maybe we should?
                                  break


                              elif c == 0 :
                                  # expansion point
                                  rn.append(temp_loc)
                                  reach_flag[ temp_loc[0], temp_loc[1] ] = 1
                                  if(net_iter) : draw_dot(win, pxscale, temp_loc)
                                  if ( signal_def[ temp_loc[0], temp_loc[1] ] <= 4 ) :
                                      signal_def[ temp_loc[0], temp_loc[1] ] = traceback
                                      #else :
                                      #print("obstruction or line-route found")
                                      if (target_found) :
                                          break;


                   if ( target_found ) :
                        break

               if ( target_found ) :
                    break

        # If RN is empty, EXIT - no connection, otherwise go to step (2).
        if rn == [] and not target_found :
           #print("no connection, RN is empty ")
           no_solution = True


    # while (target_found == False and no_solution == False ) :


    # If a target is found, trace path and remove destination dest_pins list
    if (target_found == True) :

        #print("src net end")
        #print(src_net_int)

        trace_fail, path_list_i = trace_path( signal_def, reach_flag, temp_loc, traceback )
        path_list = path_list + path_list_i
        if(trace_fail) : no_solution = True

        dest_pins = list( set( dest_pins ).difference( set( [ temp_loc ] ) ) )

    else :
        path_list = src_net_int


    return no_solution, dest_pins, path_list






def main():

    parser = argparse.ArgumentParser()

    # Add more options if you like
    parser.add_argument("-f", "--file", dest="exp_file", default="",
                        help="process specified input file")
    parser.add_argument("-s", "--step",
                        action="store_true", dest="step_iter", default=False,
                        help="step through algorithm")
    parser.add_argument("-n", "--net_step",
                        action="store_true", dest="net_iter", default=False,
                        help="step through net by net")
    parser.add_argument("-lp", "--line_probe",
                        action="store_true", dest="line_probe", default=False,
                        help="Run line-probe algorithm (default is lee-moore)")

    args = parser.parse_args()

    leemoore_run = True
    if (args.line_probe) :
        leemoore_run = False


    if (not args.exp_file) :
        example_dir = "benchmarks"
        ex_files = ["example.infile", "impossible.infile", "misty.infile", "rusty.infile", "stdcell.infile", "temp.infile", \
                    "impossible2.infile", "kuma.infile", "oswald.infile", "stanley.infile", "sydney.infile", "wavy.infile"]
    else :
        example_dir = []
        ex_files = [ args.exp_file ]

    step_iter = args.step_iter
    net_iter = args.net_iter


    for ex_file in ex_files :

        print("processing %s file" % ex_file)
        if (leemoore_run) :
            print("running Lee-Moore algorithm")
        else :
            print("running Line-Probe algorithm")

        if (not args.exp_file) :
            reader = csv.reader(open(os.path.join(example_dir, ex_file)), delimiter=" ")
        else :
            reader = csv.reader(open(ex_file), delimiter=" ")

        # display options
        colors = ['orange', 'cyan', 'green', 'blue', 'violet', 'grey', 'pink', 'brown', 'salmon', 'lime', 'aquamarine', 'blue4' ]
        pxscale = 20

        # read information about the problem from file
        xdim, ydim, obstr_loc, pin_locs = read_info(reader)

        num_segments = 0
        num_routed = 0
        path_lists = []

        #  Attempt re-ordering based on ascending # of pins in bounding box
        pin_locs = reorder_nets(pin_locs)

        for pin_group in pin_locs :

            #DE - pin_loc_i = pins not routed in this iteration
            pin_locs_i = []
            for _pin_group in pin_locs :
                if _pin_group != pin_group : pin_locs_i.append(_pin_group)

            # source pin is first input into Lee-Moore / Line-probe iteration
            src_net = [ pin_group[0] ]
            dest_pins = pin_group[1:]
            no_solution = False
            path_list = []

            num_seg_i = len(dest_pins)
            num_segments += num_seg_i

            # Run Lee-Moore iteratively, starting on source pin, then on the routed net
            while dest_pins and not no_solution:

                # draw grid, pins, and obstructions
                if (net_iter) :
                    win = draw_example(ex_file, pxscale, xdim, ydim, obstr_loc, pin_locs, colors )
                else :
                    win = []

                obstr_w_paths = obstr_loc

                for path in path_lists :
                    # add routed paths to the obstructions list
                    obstr_w_paths += path
                # draw previously routed paths

                if (net_iter) :
                    #
                    draw_path(win, pxscale, path_lists, pin_locs[:len(path_lists)], colors)
                    # draw partial path completed in previous iteration
                    draw_path(win, pxscale, [ path_list ], [ pin_group ], [ colors[ len(path_lists)%12 ] ] )


                if leemoore_run == False :

                    no_solution, dest_pins, path_list = lineprobe_mod( win, xdim, ydim, src_net, dest_pins, pin_locs_i, obstr_w_paths, pxscale, step_iter, net_iter )

                else :
                    no_solution, dest_pins, path_list = leemoore( win, xdim, ydim, src_net, dest_pins, pin_locs_i, obstr_w_paths, pxscale, step_iter, net_iter )

                if (no_solution ) :
                        path_list = src_net

                if (net_iter) :
                    win.getKey()
                    draw_path(win, pxscale, [ path_list ], [ pin_group ], [ colors[ len(path_lists)%12 ] ] )

                # for multiple sinks, path is input to next iteration of algorithm
                src_net = path_list

                if (net_iter) : win.getKey()
                if (net_iter) : win.close()


            path_lists.append(path_list)

            # how many segments were routed for this benchmark
            num_routed += ( num_seg_i - len(dest_pins) )

        # one final display
        win = draw_example(ex_file, pxscale, xdim, ydim, obstr_loc, pin_locs, colors )
        draw_path(win, pxscale, path_lists, pin_locs, colors)

        print('%d / %d segments routed' %( num_routed,  num_segments) )

        win.getKey()
        win.close()


if __name__ == "__main__":
    main()
