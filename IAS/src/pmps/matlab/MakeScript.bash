SL_ROOT=$PROG_ROOT

SL_INCLUDES="-I$PROG_ROOT/include -I$PROG_ROOT/shared/include -I$PROG_ROOT/shared/IAS/include -I$PROG_ROOT/shared/IAS/matlab"
SL_LIBS_PATHS="-L$SL_ROOT/lib -L$SL_ROOT/asdf"
SL_LIBS_PATHS_ARCH="-L$SL_ROOT/lib/x86_64 "
SL_LIBS="-lutility"


. /home/paraschos/svn_root/robolab/robolab/shared/IAS/matlab/mexopts.bash

