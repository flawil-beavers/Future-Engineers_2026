#pragma once

#include <stddef.h>
#include <stdint.h>

// The upstream transport retries address writes indefinitely and can spin on
// zero-byte chunks. Return the failure to the existing sensor-health/retry
// logic instead. The sensor register protocol and successful read order stay
// unchanged; no fabricated padding bytes are supplied after a transport fault.
namespace BoundedTofTransport {
template<class Bus>
int read(Bus &bus,uint8_t address,uint16_t reg,uint8_t *data,
         uint16_t count,size_t chunk) {
    if(!chunk || (!data && count))return 1;
    if(!count)return 0;
    const uint8_t index[]={uint8_t(reg>>8),uint8_t(reg)};
    int status=1;
    for(uint8_t attempt=0;attempt<2;++attempt){
        bus.beginTransmission(uint8_t((address>>1)&0x7F));
        if(bus.write(index,2)!=2){bus.endTransmission(true);continue;}
        status=bus.endTransmission(false);
        if(!status)break;
    }
    if(status)return 1;
    size_t offset=0;
    while(offset<count){
        const size_t size=count-offset<chunk ? count-offset : chunk;
        if(bus.requestFrom(uint8_t((address>>1)&0x7F),size)!=size)return 1;
        for(size_t i=0;i<size;++i){
            if(bus.available()<=0)return 1;
            const int value=bus.read();
            if(value<0)return 1;
            data[offset++]=uint8_t(value);
        }
    }
    return 0;
}

template<class Bus>
int write(Bus &bus,uint8_t address,uint16_t reg,uint8_t *data,
          uint16_t count,size_t chunk) {
    if(!chunk || (!data && count))return 1;
    size_t offset=0;
    while(offset<count){
        const size_t size=count-offset<chunk ? count-offset : chunk;
        const uint16_t index=reg+offset;
        const uint8_t bytes[]={uint8_t(index>>8),uint8_t(index)};
        bus.beginTransmission(uint8_t((address>>1)&0x7F));
        if(bus.write(bytes,2)!=2 || bus.write(data+offset,size)!=size){
            bus.endTransmission(true);return 1;
        }
        if(bus.endTransmission(true)!=0)return 1;
        offset+=size;
    }
    return 0;
}
} // namespace BoundedTofTransport
