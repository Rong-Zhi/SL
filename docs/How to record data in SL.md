How to record data in SL
------------------------

At every simulation cycle, SL can record current value of any global static
variable. There are just a few function calls you have to make from your
code to get access to this useful feature.

1. Set sampling time for `task_default` in `prefs/default_script`.
2. Specify what variables to record in `prefs/task_default.script`.
   A lot of commonly used variables are predefined in `YOUR_ROBOT/prefs/task_sample.script`. 
   You may also add your own variable (see below).
3. Call `scd()` to start recording data (usually at the end of
   the `task_init` function). After that, call `stopcd()` to stop
   collecting data and `saveData()` to dump recorded data to a file.
4. Go to the build folder of your robot.
   Make sure that you see files named "d{0-9}*". These are the data files.


### How to record other variables

1. Define a global `static` variable.
2. Call `addVarToCollect(YOUR_ARGUMENTS)`.
3. Call `updateDataCollectScript()` once you added all your variables.

Example (see `barrett\src\ball_following_task.c`)

```
// Add error at each timestep to the recorded variables
for (i = _X_; i <= _Z_; ++i)
	addVarToCollect((char *)&(error[i]),string,"m", DOUBLE,FALSE);
updateDataCollectScript();	
```

For more information, browse the 
[SL book](http://www-clmc.usc.edu/publications/S/schaal-TRSL.pdf).
