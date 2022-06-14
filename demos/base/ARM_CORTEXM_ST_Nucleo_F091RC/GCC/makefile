#****************************************************************************************
#|  Description: Makefile for building all demo applications in one go
#|    File Name: makefile
#|
#|---------------------------------------------------------------------------------------
#|                          C O P Y R I G H T
#|---------------------------------------------------------------------------------------
#|   Copyright (c) 2022 by Feaser     www.feaser.com     All rights reserved
#|
#|---------------------------------------------------------------------------------------
#|                            L I C E N S E
#|---------------------------------------------------------------------------------------
#| Permission is hereby granted, free of charge, to any person obtaining a copy
#| of this software and associated documentation files (the "Software"), to deal
#| in the Software without restriction, including without limitation the rights
#| to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#| copies of the Software, and to permit persons to whom the Software is
#| furnished to do so, subject to the following conditions:
#| 
#| The above copyright notice and this permission notice shall be included in all
#| copies or substantial portions of the Software.
#| 
#| THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#| IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#| FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#| AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#| LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#| OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#| SOFTWARE.
#|
#****************************************************************************************
SHELL = sh

# This makefile enables you to build and clean all the demo applications in one go:
#   - Calling "make all" builds the executable for each demo application.
#   - Calling "make clean" removes all the build created files of each demo application.
# Note that calling "make clean all" does not work. Do this in two steps. So first 
# "make clean" and then "make all".


#|--------------------------------------------------------------------------------------|
#| Collect
#|--------------------------------------------------------------------------------------|
# Each subdirectory contains the makefile of a demo application
#DEMO_DIRS := ${sort ${dir ${wildcard ./*/}}}
DEMO_DIRS := $(filter-out ./, ${sort ${dir ${wildcard ./*/}}})


#|--------------------------------------------------------------------------------------|
#| Make ALL
#|--------------------------------------------------------------------------------------|
.PHONY: all
all: $(DEMO_DIRS)


#|--------------------------------------------------------------------------------------|
#| Make CLEAN                                                                           |
#|--------------------------------------------------------------------------------------|
.PHONY: clean
clean: $(DEMO_DIRS) 


#|--------------------------------------------------------------------------------------|
#| Make specific demo
#|--------------------------------------------------------------------------------------|
.PHONY: $(DEMO_DIRS)
$(DEMO_DIRS):
	@echo $(DEMO_DIRS)   
	@echo $(MAKE) -C $@ $(MAKECMDGOALS)
#	@$(MAKE) -C $@ $(MAKECMDGOALS)




