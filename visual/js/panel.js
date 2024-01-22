/**
 * The control panel.
 */
var Panel = {
    init: function() {
        var $algo = $('#algorithm_panel');

        $('.panel').draggable();
        $('.accordion').accordion({
            collapsible: false,
        });
        $('.option_label').click(function() {
            $(this).prev().click();
        });
        $('#hide_instructions').click(function() {
            $('#instructions_panel').slideUp();
        });
        $('#play_panel').css({
            top: $algo.offset().top + $algo.outerHeight() + 20
        });
        $('#button2').attr('disabled', 'disabled');
    },
    /**
     * Get the user selected path-finder.
     * TODO: clean up this messy code.
     */
    getFinder: function() {
        var finder, selected_header, heuristic, allowDiagonal, biDirectional,
        dontCrossCorners, weight, trackRecursion, timeLimit, useMomentum, momentum,
        breakTies, ignoreStartTies, preferences = [];
        
        selected_header = $(
            '#algorithm_panel ' +
            '.ui-accordion-header[aria-selected=true]'
        ).attr('id');
        
        switch (selected_header) {

        case 'astar_header':
            allowDiagonal = typeof $('#astar_section ' +
                                     '.allow_diagonal:checked').val() !== 'undefined';
            biDirectional = typeof $('#astar_section ' +
                                     '.bi-directional:checked').val() !=='undefined';
            dontCrossCorners = typeof $('#astar_section ' +
                                     '.dont_cross_corners:checked').val() !=='undefined';
            avoidStaircase = typeof $('#astar_section ' +
                                     '.avoid_staircase:checked').val() !=='undefined';
            useMomentum      = typeof $('#astar_section ' +
                                     '.apply_momentum:checked').val() !=='undefined';
            breakTies        = typeof $('#astar_section ' +
                                    '.break_ties:checked').val() !=='undefined';
            ignoreStartTies  = typeof $('#astar_section ' +
                                    '.break_ties_startNode:checked').val() !=='undefined';

            /* parseInt returns NaN (which is falsy) if the string can't be parsed */
            turnPenalty = parseFloat($('#astar_section .turn_penalty').val()) || 0.001;
            /* values for turnPenalty should be between 0 and 1, otherwise the turnPenalty 
             * will dominate the default cost of moving between adjacent nodes. the default cost 
             * for that movement is set to 1, so it's important we stay far away
             * from that value, otherwise wonky things will happen. */
            turnPenalty = turnPenalty > 0 && turnPenalty < 1 ? turnPenalty : 0.001;

            // get the momentum value set by the user.
            // much like the turn penalty, it's best if this value is between 0 and 1, and far
            // away from the cost of moving between neighboring nodes.
            momentum = parseFloat($('#astar_section .momentum_factor').val()) || 0.0001;
            momentum = momentum > 0 && momentum < 1 ? momentum : 0.0001;

            /* parseInt returns NaN (which is falsy) if the string can't be parsed */
            weight = parseInt($('#astar_section .astar_weight').val()) || 1;
            weight = weight >= 1 ? weight : 1; /* if negative or 0, use 1 */

            // parse user-selected tie-breakers.
            if (breakTies) {

                UpRight = $('input[name=UpRight]:checked').val();
                UpDown = $('input[name=UpDown]:checked').val();
                UpLeft = $('input[name=UpLeft]:checked').val();
                RightDown = $('input[name=RightDown]:checked').val();
                LeftRight = $('input[name=LeftRight]:checked').val();
                DownLeft = $('input[name=DownLeft]:checked').val();

                preferences.push(UpRight);
                preferences.push(UpDown);
                preferences.push(UpLeft);
                preferences.push(RightDown);
                preferences.push(LeftRight);
                preferences.push(DownLeft);

                // 0 = Up, 1 = Right, 2 = Down, 3 = Left
                for (i = 0; i < preferences.length; i++) {
                    if (preferences[i] === "Up") preferences[i] = 0;
                    if (preferences[i] === "Right") preferences[i] = 1;
                    if (preferences[i] === "Down") preferences[i] = 2;
                    if (preferences[i] === "Left") preferences[i] = 3;
                    if (preferences[i] === "Ignore") preferences[i] = undefined;
                }
            }

            heuristic = $('input[name=astar_heuristic]:checked').val();
            if (biDirectional) {
                finder = new PF.BiAStarFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners,
                    heuristic: PF.Heuristic[heuristic],
                    weight: weight
                });
            } else {
                finder = new PF.AStarFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners,
                    heuristic: PF.Heuristic[heuristic],
                    weight: weight,
                    avoidStaircase: avoidStaircase,
                    turnPenalty: turnPenalty,
                    useMomentum : useMomentum,
                    momentum : momentum,
                    breakTies : breakTies,
                    ignoreStartTies : ignoreStartTies,
                    preferences : preferences
                });
            }
            break;

        case 'breadthfirst_header':
            allowDiagonal = typeof $('#breadthfirst_section ' +
                                     '.allow_diagonal:checked').val() !== 'undefined';
            biDirectional = typeof $('#breadthfirst_section ' +
                                     '.bi-directional:checked').val() !== 'undefined';
            dontCrossCorners = typeof $('#breadthfirst_section ' +
                                     '.dont_cross_corners:checked').val() !=='undefined';
            if (biDirectional) {
                finder = new PF.BiBreadthFirstFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners
                });
            } else {
                finder = new PF.BreadthFirstFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners
                });
            }
            break;

        case 'bestfirst_header':
            allowDiagonal = typeof $('#bestfirst_section ' +
                                     '.allow_diagonal:checked').val() !== 'undefined';
            biDirectional = typeof $('#bestfirst_section ' +
                                     '.bi-directional:checked').val() !== 'undefined';
            dontCrossCorners = typeof $('#bestfirst_section ' +
                                     '.dont_cross_corners:checked').val() !=='undefined';
            heuristic = $('input[name=bestfirst_heuristic]:checked').val();
            if (biDirectional) {
                finder = new PF.BiBestFirstFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners,
                    heuristic: PF.Heuristic[heuristic]
                });
            } else {
                finder = new PF.BestFirstFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners,
                    heuristic: PF.Heuristic[heuristic]
                });
            }
            break;

        case 'dijkstra_header':
            allowDiagonal = typeof $('#dijkstra_section ' +
                                     '.allow_diagonal:checked').val() !== 'undefined';
            biDirectional = typeof $('#dijkstra_section ' +
                                     '.bi-directional:checked').val() !=='undefined';
            dontCrossCorners = typeof $('#dijkstra_section ' +
                                     '.dont_cross_corners:checked').val() !=='undefined';
            if (biDirectional) {
                finder = new PF.BiDijkstraFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners
                });
            } else {
                finder = new PF.DijkstraFinder({
                    allowDiagonal: allowDiagonal,
                    dontCrossCorners: dontCrossCorners
                });
            }
            break;

        case 'jump_point_header':
            trackRecursion = typeof $('#jump_point_section ' +
                                     '.track_recursion:checked').val() !== 'undefined';
            heuristic = $('input[name=jump_point_heuristic]:checked').val();
            
            finder = new PF.JumpPointFinder({
              trackJumpRecursion: trackRecursion,
              heuristic: PF.Heuristic[heuristic],
              diagonalMovement: PF.DiagonalMovement.IfAtMostOneObstacle
            });
            break;
        case 'orth_jump_point_header':
            trackRecursion = typeof $('#orth_jump_point_section ' +
                                     '.track_recursion:checked').val() !== 'undefined';
            heuristic = $('input[name=orth_jump_point_heuristic]:checked').val();

            finder = new PF.JumpPointFinder({
              trackJumpRecursion: trackRecursion,
              heuristic: PF.Heuristic[heuristic],
              diagonalMovement: PF.DiagonalMovement.Never
            });
            break;
        case 'ida_header':
            allowDiagonal = typeof $('#ida_section ' +
                                     '.allow_diagonal:checked').val() !== 'undefined';
            dontCrossCorners = typeof $('#ida_section ' +
                                     '.dont_cross_corners:checked').val() !=='undefined';
            trackRecursion = typeof $('#ida_section ' +
                                     '.track_recursion:checked').val() !== 'undefined';

            heuristic = $('input[name=jump_point_heuristic]:checked').val();

            weight = parseInt($('#ida_section input[name=astar_weight]').val()) || 1;
            weight = weight >= 1 ? weight : 1; /* if negative or 0, use 1 */

            timeLimit = parseInt($('#ida_section input[name=time_limit]').val());

            // Any non-negative integer, indicates "forever".
            timeLimit = (timeLimit <= 0 || isNaN(timeLimit)) ? -1 : timeLimit;

            finder = new PF.IDAStarFinder({
              timeLimit: timeLimit,
              trackRecursion: trackRecursion,
              allowDiagonal: allowDiagonal,
              dontCrossCorners: dontCrossCorners,
              heuristic: PF.Heuristic[heuristic],
              weight: weight
            });

            break;
        }

        return finder;
    }
};
