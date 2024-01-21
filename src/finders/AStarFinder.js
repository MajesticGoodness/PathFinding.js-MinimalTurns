var Heap       = require('heap');
var Util       = require('../core/Util');
var Heuristic  = require('../core/Heuristic');
var DiagonalMovement = require('../core/DiagonalMovement');

/**
 * A* path-finder. Based upon https://github.com/bgrins/javascript-astar
 * @constructor
 * @param {Object} opt
 * @param {boolean} opt.allowDiagonal Whether diagonal movement is allowed.
 *     Deprecated, use diagonalMovement instead.
 * @param {boolean} opt.dontCrossCorners Disallow diagonal movement touching 
 *     block corners. Deprecated, use diagonalMovement instead.
 * @param {DiagonalMovement} opt.diagonalMovement Allowed diagonal movement.
 * @param {function} opt.heuristic Heuristic function to estimate the distance
 *     (defaults to manhattan).
 * @param {number} opt.weight Weight to apply to the heuristic to allow for
 *     suboptimal paths, in order to speed up the search.
 * @param {number} opt.avoidStaircase Add penalties to discourage turning and
 * causing a 'staircase' effect (defaults to false).
 * @param {number} opt.turnPenalty Penalty to add to turning. Higher numbers
 * discourage turning more (defaults to 0.001).
 * 
 * This value should be set to be less than the default cost of moving between
 * adjacent nodes, which is 1 in A*.  A good rule of thumb: set this to a value
 * such that the number of turns taken will never result in a total turnPenalty
 * that is greater that default move cost. That is, if you have a path which
 * took 10 turns, and your turnPenalty is 0.01, then the total of the turnPenalty
 * accrued is only 0.1, which is less than 1. You're good! No wonky things will
 * happen to the pathfinding in that case. 
 * @param {boolean} opt.breakTies Whether to prefer certain paths over others
 * when they're both tied in f-score.
 * @param {boolean} opt.ignoreStartTies Three-way ties often occur at the
 * start node, thus neccessitating an extra iteration of A*, for a total
 * of three iterations. This option lets the user ignore that tie at the start, so
 * as to avoid that extra iteration.
 * @param {array} opt.preferences The set of preferences to be used for deciding
 * how to break ties. Each element stores a direction, and its index number
 * indicates the tie case.
 * @param {number} opt.maxIterations The maximum number of iterations needed to
 * find the shortest, optimal path. (Defaults to 2) Will default to 3 if tie
 * breaking is set and we're not ignoring the ties that might occur at the
 * start node.
 * 
 * When tie breaking is used, and the beginning node isn't excluded from
 * tie resolution, it's possible for a three-way tie to occur. To accommodate
 * this, an extra iteration is needed, for a total of 3 iterations. This is
 * because A* will explore three different paths where each one had a different
 * winner for the three-way tie.
 * 
 * It may in fact find the optimal path on the very first iteration, but there's
 * no way for us to know that in advance. On the flip side, it might find the
 * suboptimal path on the first 2 iterations, only finding the optimal path on
 * the third. That's why we need run the iterations 3 times and check all 3 paths.
 * @param {boolean} opt.useMomentum Whether to apply a momentum to paths. Paths
 * which travel along the same axis will be rewarded with the momentum set
 * by the user. (Defaults to 1).
 * @param {number} opt.momentum The amount of momentum to be rewarded each time
 * a node is traveling along the same line as the previous node. Values set
 * should be smaller than your turn penalty. (Defaults to 0.0001).
 */
function AStarFinder(opt) {
    opt = opt || {};
    this.allowDiagonal = opt.allowDiagonal;
    this.dontCrossCorners = opt.dontCrossCorners;
    this.heuristic = opt.heuristic || Heuristic.manhattan;
    this.weight = opt.weight || 1;
    this.diagonalMovement = opt.diagonalMovement;
    this.avoidStaircase = opt.avoidStaircase;
    this.turnPenalty = opt.turnPenalty || 0.001;
    this.useMomentum = opt.useMomentum;
    this.momentum = opt.momentum || 0.0001;
    this.breakTies = opt.breakTies
    this.ignoreStartTies = opt.ignoreStartTies;
    this.preferences = opt.preferences;
    this.maxIterations = 2 + (this.breakTies && !this.ignoreStartTies)


    if (!this.diagonalMovement) {
        if (!this.allowDiagonal) {
            this.diagonalMovement = DiagonalMovement.Never;
        } else {
            if (this.dontCrossCorners) {
                this.diagonalMovement = DiagonalMovement.OnlyWhenNoObstacles;
            } else {
                this.diagonalMovement = DiagonalMovement.IfAtMostOneObstacle;
            }
        }
    }

    // When diagonal movement is allowed the manhattan heuristic is not
    //admissible. It should be octile instead
    if (this.diagonalMovement === DiagonalMovement.Never) {
        this.heuristic = opt.heuristic || Heuristic.manhattan;
    } else {
        this.heuristic = opt.heuristic || Heuristic.octile;
    }
}

/**
 * Find and return the the path.
 * @return {Array<Array<number>>} The path, including both start and
 *     end positions.
 */
AStarFinder.prototype.findPath = function(startX, startY, endX, endY, grid, paths, firstTileTaken, iterations) {
    var openList = new Heap(function(nodeA, nodeB) {
            return nodeA.f - nodeB.f;
        }),
        startNode = grid.getNodeAt(startX, startY),
        endNode = grid.getNodeAt(endX, endY),
        heuristic = this.heuristic,
        diagonalMovement = this.diagonalMovement,
        avoidStaircase = this.avoidStaircase,
        turnPenalty = this.turnPenalty,
        useMomentum = this.useMomentum,
        momentum = this.momentum,
        breakTies = this.breakTies,
        ignoreStartTies = this.ignoreStartTies,
        preferences = this.preferences,
        maxIterations = this.maxIterations,
        weight = this.weight,
        abs = Math.abs, SQRT2 = Math.SQRT2,
        lastDirection, node, neighbors, neighbor, i, l, x, y, ng,
        atStartNode, minFVal, iterations, dirtyNodes = [];

    // Block the first tile which was taken in the previous path to coerce A*
    // into exploring a more optimal path >:)
    if (paths) {
        grid.setWalkableAt(firstTileTaken[0], firstTileTaken[1], false);
    }

    // if this is our first iteration, then we need to initialize our variables:
    if(!iterations) {
        // keep track of which iteration we're on.
        iterations = 1;
        // store the paths A* has managed to come up with here so we can compare
        // them later
        paths = [];
    }

    // set the `g` and `f` value of the start node to be 0
    startNode.g = 0;
    startNode.f = 0;

    // keep track of which nodes have been visited so we can clean them up for our
    // subsequent iterations.
    dirtyNodes.push(startNode.y);
    dirtyNodes.push(startNode.x);

    // push the start node into the open list
    openList.push(startNode);
    startNode.opened = true;
    // Keep track of whether this is the start node so we can handle
    // ignoring tie-breaking at the start node later. we might want
    // to avoid tie-breaking at the start node because it will often
    // force another iteration of A*. we could just disable tie-breaking
    // altogether, but then we lose the benefits. so, as a compromise
    // we let the user ignore checking for ties at the start node while
    // still allowing for tie checking at all other points in the path.
    // this keeps the iterations of A* to two, while still retaining
    // the tie-breaking.
    atStartNode = true;

    // while the open list is not empty
    while (!openList.empty()) {
        // pop the position of node which has the minimum `f` value.
        node = openList.pop();
        node.closed = true;

        // if reached the end position, construct the path and return it
        if (node === endNode) {

            iterations++;

            // make a copy of this path which we found and add it to our array so
            // we can either: loop again, or, if we've reached our iteration
            // threshold, resolve which of these paths we've found is the optimal
            // one and return it.
            
            paths.push(structuredClone(endNode));

            if(iterations > maxIterations) {
                // which of these paths we've found is the best one?
                let bestPathFound = bestPath(paths);
                // whichever that one may happen to be, return it.
                return Util.backtrace(bestPathFound);
            }

            // Give us back the path we found in as an array of its coordinates.
            thisPath = Util.backtrace(endNode);

            // if our path's lengths turned out to be just 2 nodes, then our goal
            // is directly next to us. in that case, there's no need to iterate
            // again, as the optimal path is guaranteed to be found on the first loop.

            if(thisPath.length === 2) return thisPath;
            firstTileTaken = thisPath[1];

            // reset all the nodes we visited this iteration back to an untouched state
            // before we loop again.

            grid.cleanUp(dirtyNodes, firstTileTaken);
            return this.findPath(startX, startY, endX, endY, grid, paths, firstTileTaken, iterations);
        }

        // get neigbours of the current node
        neighbors = grid.getNeighbors(node, diagonalMovement);

        // the nodes which we're going break our ties on.
        let neighborsAddedToList = [];

        // keep track of the previous f-score so we can find the lowest f-score among
        // the nodes being considered. nodes which have the same f-score are considered
        // to be tied.
        let prevFVal = undefined;
        
        for (i = 0, l = neighbors.length; i < l; ++i) {
            neighbor = neighbors[i];

            if (neighbor.closed) {
                continue;
            }

            x = neighbor.x;
            y = neighbor.y;

            // get the distance between current node and the neighbor
            // and calculate the next g score
            ng = node.g + ((x - node.x === 0 || y - node.y === 0) ? 1 : SQRT2);

            // if we're avoiding staircasing, add penalties if the direction 
            // will change
            if (avoidStaircase) {
                lastDirection = node.parent === undefined? undefined : { x : node.x - node.parent.x, y : node.y - node.parent.y };
                var turned = lastDirection === undefined? 0 : lastDirection.x !== x - node.x || lastDirection.y !== y - node.y;
                ng += turnPenalty * turned;

                // store the last time we turned.
                if(turned) {
                    neighbor.lastTurn = true;
                    neighbor.lastTurnX = node.x;
                    neighbor.lastTurnY = node.y;
                }

                else {
                    // pass the last time we turned to the next node so we can
                    // use that information to reset our momentum.
                    if(node.lastTurn = true) {
                        neighbor.lastTurnX = node.lastTurnX;
                        neighbor.lastTurnY = node.lastTurnY;
                    }
                }
                // reward paths that have some momentum going.
                ng -= momentum * !turned * useMomentum
                
                // reset our momentum when we turn.
                ng += momentum * turned * 
                heuristic(abs(node.x - neighbor.lastTurnX), abs(node.y - neighbor.lastTurnY)) 
                * useMomentum;
            }

            // check if the neighbor has not been inspected yet, or
            // can be reached with smaller cost from the current node
            if (!neighbor.opened || ng < neighbor.g) {
                neighbor.g = ng;
                neighbor.h = neighbor.h || weight * heuristic(abs(x - endX), abs(y - endY));
                neighbor.f = neighbor.g + neighbor.h;
                neighbor.parent = node;

                // if the previous f score is greater than the current one,
                // then replace the minimum f score we've seen with this one.

                if (prevFVal > neighbor.f || prevFVal == undefined) {
                    minFVal = neighbor.f;
                }
                prevFVal = neighbor.f;

                if (!neighbor.opened) {
                    openList.push(neighbor);
                    neighbor.opened = true;
                    // Add nodes being considered to this array so we can pass it to resolveTies().
                    neighborsAddedToList.push(neighbor);
                    // Nodes which we need to clean up later for our next iteration of A*.
                    dirtyNodes.push(neighbor.y);
                    dirtyNodes.push(neighbor.x);
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    openList.updateItem(neighbor);
                }
            }
        } // end for each neighbor

        // Handle tie-breaking for all moves except for those that occur at the start node.
        if (breakTies && ignoreStartTies && !atStartNode) {
            resolveTies(neighborsAddedToList, minFVal, preferences);
        }

        // Include the start node in tie-breaking. This will result in an extra iteration of A*.
        // The start often result in three-way ties, so we'll need to check all three possible
        // paths, bringing us to a total of 3 iterations needed to find the optimal path.
        
        else if (breakTies) {
            resolveTies(neighborsAddedToList, minFVal, preferences);
        }
        // We're no longer at the start node after this loop completes.
        atStartNode = false;

    } // end while not open list empty

    // if this iteration resulted in no path found, just return the best path
    // found up until this point.
    
    if(paths) {
        let bestPathFound = bestPath(paths);
        return Util.backtrace(bestPathFound);
    }

    // fail to find the path
    return [];
};

module.exports = AStarFinder;
