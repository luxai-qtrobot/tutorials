--
-- Authors: Ali Paikan
-- Version: 1.0.0
-- The QTrobot look around
--
return rfsm.state {


    --States

    --Idle
    Idle = rfsm.state{

        --Bored
        Bored = rfsm.state{
            doo = function()   end,
        }, --end of Bored


        --LookAround
        LookAround = rfsm.state{
            doo = function()   end,
        }, --end of LookAround


        initial = rfsm.conn{ },
    }, --end of Idle


    --Suspend
    Suspend = rfsm.state{
        doo = function()   end,
    }, --end of Suspend


    initial = rfsm.conn{ },


    --Transitions
    rfsm.trans{ src = 'initial', tgt = 'Idle.initial', pn = 0 },
    rfsm.trans{ src = 'Idle.initial', tgt = 'Idle.LookAround', pn = 0 },
    rfsm.trans{ src = 'Idle.LookAround', tgt = 'Idle.Bored', pn = 0, events = {"e_bored"} },
    rfsm.trans{ src = 'Idle.Bored', tgt = 'Idle.LookAround', pn = 0 },
    rfsm.trans{ src = 'Idle', tgt = 'Suspend', pn = 0, events = {"e_suspend"} },
    rfsm.trans{ src = 'Suspend', tgt = 'Idle.initial', pn = 0, events = {"e_resume"} },
}
