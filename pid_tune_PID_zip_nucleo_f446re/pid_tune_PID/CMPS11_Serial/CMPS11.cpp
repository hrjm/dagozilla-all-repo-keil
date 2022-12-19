#include "CMPS11.h"

cmps11::cmps11(PinName tx, PinName rx ) : _cmps11(tx,rx) 
{
    readnow = 0 ;
    readlast = 0 ; 
    sudut = 0 ;
    modelast = 0 ;
    modenow = 0 ;
    _cmps11.format(8,SerialBase::None,2);
}

char cmps11::startCalibration()
{
    _cmps11.putc(0xF0);
    while( !(_cmps11.readable()))
    {}
    if( _cmps11.getc() != 0x55 )
    {
        return 0 ; 
    }
    
    _cmps11.putc(0xF5);
    while( !(_cmps11.readable()))
    {}
    if( _cmps11.getc() != 0x55 )
    {
        return 0 ; 
    }
    
    _cmps11.putc(0xF7);
    while( !(_cmps11.readable()))
    {}
    if( _cmps11.getc() != 0x55 )
    {
        return 0 ; 
    }
    return 1 ;
}

char cmps11::stopCalibration()
{
    _cmps11.putc(0xF8);   
    while( !(_cmps11.readable()))
    {}
    if( _cmps11.getc() != 0x55 )
    {
        return 0 ; 
    }
    cmps11::set0degree() ;
    return 1 ;
}

int cmps11::readCompassAngle16Bit()
{
    _cmps11.putc(0x13);
    while ( !(_cmps11.readable()) ) ;
    buffer[0] = _cmps11.getc() ;
    while ( !(_cmps11.readable()) ) ;
    buffer[1] = _cmps11.getc() ;
    
    return (  ( buffer[0] << 8 ) | buffer[1]  ) ; 

}

void cmps11::set0degree()
{
    sudut = 0 ;
    readnow = cmps11::readCompassAngle16Bit() ; 
    readlast = readnow ;   
}

int cmps11::readAngle16Bit()
{
    readnow = cmps11::readCompassAngle16Bit() ;
    bufferx = readnow-readlast ;
    readlast = readnow ;
    sudut += bufferx ; 
            
    if ( (( sudut < 0 ) || ( sudut > 3599 )))
    {
        modenow = 1 ;
    }
    else
    {
        modenow = 0 ;   
    }
            
    if ( modelast != modenow )
    {
        if ( sudut < 0 )
        {
            sudut += 3599 ;   
        }
        else if ( sudut > 3599 )
        {
            sudut -= 3599 ;       
        }
        modelast = modenow ;
    }
    return ( sudut );              
}