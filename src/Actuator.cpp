//=============================================================================
//
//  Copyright(c) 2012 Jean Inderchit All rights reserved.
//
//  These coded instructions, statements, and computer programs contain
//  unpublished proprietary information written by Jean Inderchit and
//  are protected by copyright law. They may not be disclosed
//  to third parties or copied or duplicated in any form, in whole or
//  in part, without the prior written consent of Jean Inderchit.
//
//=============================================================================


//=============================================================================
// I N C L U D E   F I L E S

#include "Actuator.h"
#include <Arduino.h>


//=============================================================================
// C O N S T A N T (S)   &   L O C A L   C O D E


//=============================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//-----------------------------------------------------------------------------
//
Actuator::Actuator( int pinE, int pin1, int pin2 )
    : pinE_( pinE )
    , pin1_( pin1 )
    , pin2_( pin2 )
{

}

//-----------------------------------------------------------------------------
//
Actuator::~Actuator()
{
}


//=============================================================================
// M E T H O D S   C O D E   S E C T I O N

//-----------------------------------------------------------------------------
//
void
Actuator::Initialize() const
{
    pinMode( pinE_, OUTPUT );
    pinMode( pin1_, OUTPUT );
    pinMode( pin2_, OUTPUT );
}

//-----------------------------------------------------------------------------
//
void
Actuator::Activate() const
{
    digitalWrite( pinE_, HIGH );
}

//-----------------------------------------------------------------------------
//
void
Actuator::Deactivate() const
{
    digitalWrite( pinE_, LOW );
}

//-----------------------------------------------------------------------------
//
void
Actuator::Coast() const
{
    digitalWrite( pin1_, LOW );
    digitalWrite( pin2_, LOW );
}

//-----------------------------------------------------------------------------
//
void
Actuator::Brake() const
{
    digitalWrite( pin1_, HIGH );
    digitalWrite( pin2_, HIGH );
}

//-----------------------------------------------------------------------------
//
void
Actuator::Forward( const unsigned int val )
{
    int rval = 54 + val*201/100;

    // digitalWrite(pin1_, LOW);
    // digitalWrite(pin2_, HIGH);
    // analogWrite(pinE_,rval);

    analogWrite( pin1_, 0 );
    analogWrite( pin2_, rval );
}

//-----------------------------------------------------------------------------
//
void
Actuator::Backward( const unsigned int val )
{
    int rval = 54 + val*201/100;

    // digitalWrite(pin1_, HIGH);
    // digitalWrite(pin2_, LOW);
    // analogWrite(pinE_, 10);

    analogWrite( pin1_, rval );
    analogWrite( pin2_, 0 );
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-------------------------------- UNIT TESTS ---------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


//=============================================================================
// U N I T   T E S T   C O D E   S E C T I O N
