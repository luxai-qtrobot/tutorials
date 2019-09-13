--
-- Authors: Ali Paikan
-- Version: 1.0.0
-- QTrobot memmory game state machine 
--
return rfsm.state {


    --States

    --Suspend
    Suspend = rfsm.state{
        doo = function()   end,
    }, --end of Suspend


    --Waiting
    Waiting = rfsm.state{
        doo = function()   end,
    }, --end of Waiting


    initial = rfsm.conn{ },

    --Game
    Game = rfsm.state{

        initial = rfsm.conn{ },

        --Play
        Play = rfsm.state{
            doo = function()   end,
        }, --end of Play

    }, --end of Game



    --Transitions
    rfsm.trans{ src = 'initial', tgt = 'Waiting', pn = 0 },
    rfsm.trans{ src = 'Waiting', tgt = 'Game.initial', pn = 0, events = {"e_start_game"} },
    rfsm.trans{ src = 'Game', tgt = 'Suspend', pn = 0, events = {"e_suspend"} },
    rfsm.trans{ src = 'Suspend', tgt = 'Game.initial', pn = 0, events = {"e_resume"} },
    rfsm.trans{ src = 'Game.Play', tgt = 'Waiting', pn = 0 },
    rfsm.trans{ src = 'Game.initial', tgt = 'Game.Play', pn = 0 },
}
