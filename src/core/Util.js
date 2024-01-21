/**
 * Backtrace according to the parent records and return the path.
 * (including both start and end nodes)
 * @param {Node} node End node
 * @return {Array<Array<number>>} the path
 */
function backtrace(node) {
    var path = [[node.x, node.y]];
    while (node.parent) {
        node = node.parent;
        path.push([node.x, node.y]);
    }
    return path.reverse();
}
exports.backtrace = backtrace;

/**
 * Backtrace from start and end node, and return the path.
 * (including both start and end nodes)
 * @param {Node}
 * @param {Node}
 */
function biBacktrace(nodeA, nodeB) {
    var pathA = backtrace(nodeA),
        pathB = backtrace(nodeB);
    return pathA.concat(pathB.reverse());
}
exports.biBacktrace = biBacktrace;

/**
 * Compute the length of the path.
 * @param {Array<Array<number>>} path The path
 * @return {number} The length of the path
 */
function pathLength(path) {
    var i, sum = 0, a, b, dx, dy;
    for (i = 1; i < path.length; ++i) {
        a = path[i - 1];
        b = path[i];
        dx = a[0] - b[0];
        dy = a[1] - b[1];
        sum += Math.sqrt(dx * dx + dy * dy);
    }
    return sum;
}
exports.pathLength = pathLength;


/**
 * Given the start and end coordinates, return all the coordinates lying
 * on the line formed by these coordinates, based on Bresenham's algorithm.
 * http://en.wikipedia.org/wiki/Bresenham's_line_algorithm#Simplification
 * @param {number} x0 Start x coordinate
 * @param {number} y0 Start y coordinate
 * @param {number} x1 End x coordinate
 * @param {number} y1 End y coordinate
 * @return {Array<Array<number>>} The coordinates on the line
 */
function interpolate(x0, y0, x1, y1) {
    var abs = Math.abs,
        line = [],
        sx, sy, dx, dy, err, e2;

    dx = abs(x1 - x0);
    dy = abs(y1 - y0);

    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;

    err = dx - dy;

    while (true) {
        line.push([x0, y0]);

        if (x0 === x1 && y0 === y1) {
            break;
        }
        
        e2 = 2 * err;
        if (e2 > -dy) {
            err = err - dy;
            x0 = x0 + sx;
        }
        if (e2 < dx) {
            err = err + dx;
            y0 = y0 + sy;
        }
    }

    return line;
}
exports.interpolate = interpolate;


/**
 * Given a compressed path, return a new path that has all the segments
 * in it interpolated.
 * @param {Array<Array<number>>} path The path
 * @return {Array<Array<number>>} expanded path
 */
function expandPath(path) {
    var expanded = [],
        len = path.length,
        coord0, coord1,
        interpolated,
        interpolatedLen,
        i, j;

    if (len < 2) {
        return expanded;
    }

    for (i = 0; i < len - 1; ++i) {
        coord0 = path[i];
        coord1 = path[i + 1];

        interpolated = interpolate(coord0[0], coord0[1], coord1[0], coord1[1]);
        interpolatedLen = interpolated.length;
        for (j = 0; j < interpolatedLen - 1; ++j) {
            expanded.push(interpolated[j]);
        }
    }
    expanded.push(path[len - 1]);

    return expanded;
}
exports.expandPath = expandPath;


/**
 * Smoothen the give path.
 * The original path will not be modified; a new path will be returned.
 * @param {PF.Grid} grid
 * @param {Array<Array<number>>} path The path
 */
function smoothenPath(grid, path) {
    var len = path.length,
        x0 = path[0][0],        // path start x
        y0 = path[0][1],        // path start y
        x1 = path[len - 1][0],  // path end x
        y1 = path[len - 1][1],  // path end y
        sx, sy,                 // current start coordinate
        ex, ey,                 // current end coordinate
        newPath,
        i, j, coord, line, testCoord, blocked;

    sx = x0;
    sy = y0;
    newPath = [[sx, sy]];

    for (i = 2; i < len; ++i) {
        coord = path[i];
        ex = coord[0];
        ey = coord[1];
        line = interpolate(sx, sy, ex, ey);

        blocked = false;
        for (j = 1; j < line.length; ++j) {
            testCoord = line[j];

            if (!grid.isWalkableAt(testCoord[0], testCoord[1])) {
                blocked = true;
                break;
            }
        }
        if (blocked) {
            lastValidCoord = path[i - 1];
            newPath.push(lastValidCoord);
            sx = lastValidCoord[0];
            sy = lastValidCoord[1];
        }
    }
    newPath.push([x1, y1]);

    return newPath;
}
exports.smoothenPath = smoothenPath;


/**
 * Compress a path, remove redundant nodes without altering the shape
 * The original path is not modified
 * @param {Array<Array<number>>} path The path
 * @return {Array<Array<number>>} The compressed path
 */
function compressPath(path) {

    // nothing to compress
    if(path.length < 3) {
        return path;
    }

    var compressed = [],
        sx = path[0][0], // start x
        sy = path[0][1], // start y
        px = path[1][0], // second point x
        py = path[1][1], // second point y
        dx = px - sx, // direction between the two points
        dy = py - sy, // direction between the two points
        lx, ly,
        ldx, ldy,
        sq, i;

    // normalize the direction
    sq = Math.sqrt(dx*dx + dy*dy);
    dx /= sq;
    dy /= sq;

    // start the new path
    compressed.push([sx,sy]);

    for(i = 2; i < path.length; i++) {

        // store the last point
        lx = px;
        ly = py;

        // store the last direction
        ldx = dx;
        ldy = dy;

        // next point
        px = path[i][0];
        py = path[i][1];

        // next direction
        dx = px - lx;
        dy = py - ly;

        // normalize
        sq = Math.sqrt(dx*dx + dy*dy);
        dx /= sq;
        dy /= sq;

        // if the direction has changed, store the point
        if ( dx !== ldx || dy !== ldy ) {
            compressed.push([lx,ly]);
        }
    }

    // store the last point
    compressed.push([px,py]);

    return compressed;
}
exports.compressPath = compressPath;

/**
 * Given an array of endNode, returns the one with the lowest f-score.
 * @param {Array<endNode>} paths The array of endNodes.
 * @return {endNode} The end node with the lowest f-score.
 */
function bestPath(paths) {
    currentBestPath = paths[0];

    for(i = 1; i < paths.length; i++) {
        if(paths[i].f < currentBestPath.f) {
            currentBestPath = paths[i];
        }
    }
    return currentBestPath;
}
exports.bestPath = bestPath;

/**
 * Resolve ties between neighbor nodes.
 * @param {Array<neighbor>} neighborsAddedToList The array filled with neighbors to the parent node.
 * @param {number} minFVal The lowest f score which a neighbor contains. Will be used to detect ties.
 * @param {Array<number>} preferences An array which contains which directions should win ties.
 * 
 * Who should win?
 * ↑ vs → case 0 
 * ↑ vs ↓ case 1
 * ↑ vs ← case 2
 * → vs ↓ case 3
 * ← vs → case 4
 * ↓ vs ← case 5
 * 
 * Directions: ↑ = 0, → = 1, ↓ = 2, ← = 3
 * 
 * Example:
 * 
 * preferences = [0, 0, 0, 2, 1, 2], then
 * Up wins over Right.
 * Up wins over Down.
 * Up wins over Left.
 * Down wins over Right.
 * Right wins over Left.
 * Down wins over Left.
 * 
 */
function resolveTies(neighborsAddedToList, minFVal, preferences) {
    tieExists = false;
    // be sure to set TIE_BREAKER to something less than your turn penalty or
    // wonky things will happen. don't ask me how i know :(
    TIE_BREAKER = turnPenalty / 100;
    tieCase = undefined;
    threeWayTie = false;

    for (i = 0, j = 0; i < neighborsAddedToList.length; i++) {
        if (minFVal === neighborsAddedToList[i].f) {
            j++;
        }
        // if we have a three way tie, then j will be greater than 2.
        // we'll handle that by only considering the tie which is
        // traveling in the same direction. 
        if (j > 2) {
            threeWayTie = true;
            break;
        }
    }

    if (threeWayTie) {

        for (i = 0; i < neighborsAddedToList.length; i++) {
            if (tieExists) break;

            for (j = i + 1; j < neighborsAddedToList.length; j++) {

                if ((neighborsAddedToList[i].f === neighborsAddedToList[j].f)
                    && (neighborsAddedToList[i].f === minFVal)
                    // in the case of three-way ties, neighbors being tied should share the same x or y.
                    && (neighborsAddedToList[i].x === neighborsAddedToList[j].x
                        || neighborsAddedToList[i].y === neighborsAddedToList[j].y)) {
                    neighborA = neighborsAddedToList[i];
                    neighborB = neighborsAddedToList[j];
                    tieExists = true;
                    break;
                }
            }
        }
    }

    else {

        for (i = 0; i < neighborsAddedToList.length; i++) {
            if (tieExists) break;

            for (j = i + 1; j < neighborsAddedToList.length; j++) {

                if ((neighborsAddedToList[i].f === neighborsAddedToList[j].f)
                    && (neighborsAddedToList[i].f === minFVal)) {
                    neighborA = neighborsAddedToList[i];
                    neighborB = neighborsAddedToList[j];
                    tieExists = true;
                    break;
                }
            }
        }

    }

    if (!tieExists) {
        return;
    }

    if ((neighborA.direction === 0 && neighborB.direction === 1)
        || (neighborA.direction === 1 && neighborB.direction === 0)) {
        tieCase = 0
    }
    if ((neighborA.direction === 0 && neighborB.direction === 2)
        || (neighborA.direction === 2 && neighborB.direction === 0)) {
        tieCase = 1
    }
    if ((neighborA.direction === 0 && neighborB.direction === 3)
        || (neighborA.direction === 3 && neighborB.direction === 0)) {
        tieCase = 2
    }
    if ((neighborA.direction === 1 && neighborB.direction === 2)
        || (neighborA.direction === 2 && neighborB.direction === 1)) {
        tieCase = 3
    }
    if ((neighborA.direction === 3 && neighborB.direction === 1)
        || (neighborA.direction === 1 && neighborB.direction === 3)) {
        tieCase = 4
    }
    if ((neighborA.direction === 2 && neighborB.direction === 3)
        || (neighborA.direction === 3 && neighborB.direction === 2)) {
        tieCase = 5
    }

    switch (tieCase) {
        case 0:
            if (neighborA.direction === preferences[tieCase]) neighborA.g -= TIE_BREAKER;
            else if (neighborB.direction === preferences[tieCase]) neighborB.g -= TIE_BREAKER;
            break;
        case 1:
            if (neighborA.direction === preferences[tieCase]) neighborA.g -= TIE_BREAKER;
            else if (neighborB.direction === preferences[tieCase]) neighborB.g -= TIE_BREAKER;
            break;
        case 2:
            if (neighborA.direction === preferences[tieCase]) neighborA.g -= TIE_BREAKER;
            else if (neighborB.direction === preferences[tieCase]) neighborB.g -= TIE_BREAKER;
            break;
        case 3:
            if (neighborA.direction === preferences[tieCase]) neighborA.g -= TIE_BREAKER;
            else if (neighborB.direction === preferences[tieCase]) neighborB.g -= TIE_BREAKER;
            break;
        case 4:
            if (neighborA.direction === preferences[tieCase]) neighborA.g -= TIE_BREAKER;
            else if (neighborB.direction === preferences[tieCase]) neighborB.g -= TIE_BREAKER;
            break;
        case 5:
            if (neighborA.direction === preferences[tieCase]) neighborA.g -= TIE_BREAKER;
            else if (neighborB.direction === preferences[tieCase]) neighborB.g -= TIE_BREAKER;
            break;
        default:
            return;
    }
}

exports.resolveTies = resolveTies;
