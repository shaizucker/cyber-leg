#include "slcan_frame.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

int main() {
    slcan::Frame frame;
    char output[32];
    assert(slcan::parse("t0000",frame)); // ID zero must not be dropped.
    assert(frame.id==0 && frame.len==0 && !frame.extended);
    assert(slcan::format(frame,output)==6 && strcmp(output,"t0000\r")==0);
    assert(slcan::parse("t6018670B000000000476",frame));
    assert(frame.id==0x601 && frame.len==8 && frame.data[7]==0x76);
    slcan::format(frame,output);
    assert(strcmp(output,"t6018670B000000000476\r")==0);
    assert(slcan::parse("T1fffffff2aBcD",frame));
    assert(frame.extended && frame.id==0x1fffffff && frame.data[0]==0xab && frame.data[1]==0xcd);
    slcan::format(frame,output); assert(strcmp(output,"T1FFFFFFF2ABCD\r")==0);
    assert(slcan::parse("r1238",frame));
    assert(frame.remote && frame.len==8 && !frame.extended);
    slcan::format(frame,output); assert(strcmp(output,"r1238\r")==0);
    assert(slcan::parse("R000001238",frame));
    assert(frame.remote && frame.extended && frame.id==0x123);
    slcan::format(frame,output); assert(strcmp(output,"R000001238\r")==0);
    const char *bad[]={"","t","t123","t8000","T200000000","t1239","t123G",
        "t1231","t12310011","t1231GG","r123100","t12z0","b1230","O"};
    for(const char *line:bad) assert(!slcan::parse(line,frame));
    puts("SLCAN protocol checks passed: standard/extended/RTR, ID zero, DLC, strict lengths and malformed input.");
}
