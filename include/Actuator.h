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

#ifndef ACTUATOR_H
#define ACTUATOR_H

//=============================================================================
// I N C L U D E   F I L E S

//=============================================================================
// F O R W A R D   D E C L A R A T I O N S

//=============================================================================
// C O N S T A N T S

//=============================================================================
// C L A S S E S

class Actuator
{
private:
protected:
public:
    Actuator( int pinE, int pin1, int pin2 );
    ~Actuator();

    void Initialize() const;
    void Activate() const;
    void Deactivate() const;

    void Coast() const;
    void Brake() const;
    void Forward( const unsigned int val );
    void Backward( const unsigned int val );

    // Data members
private:
    int pinE_;
    int pin1_;
    int pin2_;
};

//=============================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
