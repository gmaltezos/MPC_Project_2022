% Read content from the given url with specified timeout
%
% [DATA, TIMEDOUT] = readWithTimeout(URL, TIMEOUT) reads the content 
% from the given URL and returns it to the character array DATA. 
% If the operation did not finish before the default timeout 
% the operation returns true in TIMEDOUT
%
% [DATA, TIMEDOUT] = readWithTimeout(URL, TIMEOUT) reads the content 
% from the given URL and returns it to the character array DATA. 
% If the operation did not finish before the set TIMEOUT (in sec) 
% the operation returns true in TIMEDOUT
%
% See also ForcesWeb read download write notFoundException
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.
