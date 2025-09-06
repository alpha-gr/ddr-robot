%====================================================================================
% robotqak description   
%====================================================================================
request( engage, engage(OWNER,STEPTIME) ).
reply( engagedone, engagedone(ARG) ).  %%for engage
reply( engagerefused, engagerefused(ARG) ).  %%for engage
dispatch( disengage, disengage(ARG) ).
request( pickUpCargo, pickUpCargo(SLOT) ).
reply( pickUpDone, pickUpDone(SLOT) ).  %%for pickUpCargo
request( moverobot, moverobot(TARGETX,TARGETY) ).
reply( moverobotdone, moverobotok(ARG) ).  %%for moverobot
reply( moverobotfailed, moverobotfailed(PLANDONE,PLANTODO) ).  %%for moverobot
event( alarm, alarm(X) ).
dispatch( dropDone, dropDone(SLOT) ).
dispatch( setdirection, dir(D) ). %D =up|down!left|right
request( getenvmap, getenvmap(X) ).
reply( envmap, envmap(MAP) ).  %%for getenvmap
dispatch( cmd, cmd(MOVE) ). %MOVE = a|d|l|r|h   
dispatch( setrobotstate, setpos(X,Y,D) ).
%====================================================================================
context(ctxbasicrobot, "localhost",  "TCP", "8020").
 qactor( basicrobot, ctxbasicrobot, "it.unibo.basicrobot.Basicrobot").
 static(basicrobot).
