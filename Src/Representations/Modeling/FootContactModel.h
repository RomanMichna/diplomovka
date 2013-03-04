/*
 * FootContactModel.h
 *
 *  Created on: Mar 14, 2012
 *      Author: arneboe@tzi.de
 *              simont@tzi.de
 */

#pragma once

#include "Tools/Streams/Streamable.h"


class FootContactModel : public Streamable
{
public:
    bool contactLeft;               /** < do we have foot contact with the left foot? */
    bool contactRight;              /** < do we have foot contact with the right foot? */
    int contactDurationLeft;        /** < duration (in frames) of the current contact of the left foot. 0 if no contact */
    int contactDurationRight;       /** < duration (in frames) of the current contact of the right foot. 0 if no contact */
    unsigned lastContactLeft;       /** < timestamp of the last contact detection of the left foot */
    unsigned lastContactRight;      /** < timestamp of the last contact detection of the right foot */

    FootContactModel()
    {
      contactLeft = contactRight = false;
      contactDurationLeft = contactDurationRight = lastContactLeft = lastContactRight = 0;
    }

private:
    /** Streaming function
    * @param in Object for streaming in the one direction
    * @param out Object for streaming in the other direction
    */
    virtual void serialize(In* in, Out* out)
    {
        STREAM_REGISTER_BEGIN;
          STREAM(contactLeft);
          STREAM(contactRight);
          STREAM(contactDurationLeft);
          STREAM(contactDurationRight);
          STREAM(lastContactRight);
          STREAM(lastContactLeft);
        STREAM_REGISTER_FINISH;
    }
};
