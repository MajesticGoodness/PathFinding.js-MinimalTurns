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
AStarFinder.prototype.findPath = function(startX, startY, endX, endY, grid) {
    var openList = new Heap(function(nodeA, nodeB) {
            return nodeA.f - nodeB.f;
        }),
        startNode = grid.getNodeAt(startX, startY),
        endNode = grid.getNodeAt(endX, endY),
        heuristic = this.heuristic,
        diagonalMovement = this.diagonalMovement,
        avoidStaircase = this.avoidStaircase,
        turnPenalty = this.turnPenalty,
        weight = this.weight,
        abs = Math.abs, SQRT2 = Math.SQRT2,
        lastDirection, node, neighbors, neighbor, i, l, x, y, ng;

    // set the `g` and `f` value of the start node to be 0
    startNode.g = 0;
    startNode.f = 0;

    // push the start node into the open list
    openList.push(startNode);
    startNode.opened = true;

    // while the open list is not empty
    while (!openList.empty()) {
        // pop the position of node which has the minimum `f` value.
        node = openList.pop();
        node.closed = true;

        // if reached the end position, construct the path and return it
        if (node === endNode) {
            return Util.backtrace(endNode);
        }

        // get neigbours of the current node
        neighbors = grid.getNeighbors(node, diagonalMovement);
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
            }

            // check if the neighbor has not been inspected yet, or
            // can be reached with smaller cost from the current node
            if (!neighbor.opened || ng < neighbor.g) {
                neighbor.g = ng;
                neighbor.h = neighbor.h || weight * heuristic(abs(x - endX), abs(y - endY));
                neighbor.f = neighbor.g + neighbor.h;
                neighbor.parent = node;

                if (!neighbor.opened) {
                    openList.push(neighbor);
                    neighbor.opened = true;
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    openList.updateItem(neighbor);
                }
            }
        } // end for each neighbor
    } // end while not open list empty

    // fail to find the path
    return [];
};

module.exports = AStarFinder;
