//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// This is a simplified error handler.
// This error handler takes the error code and passes it to the GetErrorText()
// procedure along with a buffer to place an error message string.
// This error message string can then be output to a user display device
// like the console
// Specific error codes should be parsed depending on the application.
//

#include "TrakSTAR.h"
#include "stdafx.h"
#include "stdio.h"
#include "string.h"
#include <Windows.h>
#include <qDebug.h>

void errorHandler(int error)
{
	char			buffer[1024];
	char			*pBuffer = &buffer[0];
	int				numberBytes;
	int				currentError = error;
	int				nextError;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
//  The following code shows you how to use the procedure GetErrorText().
//
//  When making the call you will pass the errorcode to be decoded and
//  a pointer to a buffer where the message string will be placed
//  Note: This procedure like all the others will also return an error code.
//  This new error code will either indicate a problem with the call itself or
//  will simply be the next error code in the system error code queue.
//  Looping on a test of this returned error code will cause the 
//  extraction of all current errors in the queue.
//
	do{
		nextError = GetErrorText(currentError, pBuffer, sizeof(buffer), SIMPLE_MESSAGE);

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
//	Display the message string for the "current error"
//
//  Insert display mechanism of choice here. As an example this sample
//	simply sends the message string to the console display using printf()
//	Note: The message strings returned from the call do not contain a
//	terminating newline character. If the user needs the strings to be
//	displayed on succeeding lines then a newline "\n" needs to be added.
//
		numberBytes = strlen(buffer);
		buffer[numberBytes] = '\n';		// append a newline to buffer
        qDebug("%s", buffer);

		currentError = nextError;
	}while(currentError!=BIRD_ERROR_SUCCESS);

	exit(0);
}


