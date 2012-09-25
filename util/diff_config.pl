# Copyright (c) 2012 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Uri Wiener
#

# Script which takes two config.ini files and generates a semantic diff. The
# resulting diff shows which parts of the configurations differed, and in the
# case that there is a difference it displays it. This allows rapid comparision
# of two gem5 runs and therefore provides an easy method to ensure that
# configurations are similar, or not.

#!/usr/bin/perl
use strict;

die "Please check args... " unless ($#ARGV == 1);
my $config1FileName = $ARGV[0];
my $config2FileName = $ARGV[1];

# Get just the name of the file, rather than the full path
my $config1ShortName = getFilenameFromPath($config1FileName);
my $config2ShortName = getFilenameFromPath($config2FileName);

# If the file names are the same, use the full path
if ($config1ShortName == $config2ShortName) {
    $config1ShortName = $config1FileName;
    $config2ShortName = $config2FileName;
}

print "\nComparing the following files:\n",
    "\t$config1FileName\n",
    "\tvs.\n",
    "\t$config2FileName\n\n";

# Read in the two config files
my %config1 = readConfig($config1FileName);
my %config2 = readConfig($config2FileName);

# Compare the two config files. For the first comparision we also compare the
# values (setting the first parameter to 1). There is little point doing this
# for the second comparison as it will yield the same information.
compareConfigs( 1, \%config1, $config1ShortName, \%config2, $config2ShortName );
compareConfigs( 0, \%config2, $config2ShortName, \%config1, $config1ShortName );


########################################################
# Compare values and return unique values
########################################################
sub compareValues {
    my $values1 = shift;
    my $values2 = shift;
    my @splitValues1 = split(/ /, $values1);
    my @splitValues2 = split(/ /, $values2);
    my @uniqueValues;

    foreach my $val1 (@splitValues1) {
        my $foundMatch = 0;

        # if both values equal set match flag, then break loop
        foreach my $val2 (@splitValues2) {
            if ($val1 eq $val2) {
                $foundMatch = 1;
                last;
            }

            # in case of ports, ignore port number and match port name only
            if ($val1 =~ /\[/ and $val2 =~ /\[/) {
                $val1 =~ m/^(.*)\[.*\]/;
                my $val1Name = $1;
                $val2 =~ m/^(.*)\[.*\]/;
                my $val2Name = $1;

                # if both values equal set match flag, then break loop
                if ($val1Name eq $val2Name) {
                    $foundMatch = 1;
                    last;
                }
            }
        }

        # Otherwise, the value is unique.
        if (not $foundMatch) {
            push(@uniqueValues, $val1);
        }
    }

    return join(", ", @uniqueValues);
}


########################################################
# Compare two config files. Print differences.
########################################################
sub compareConfigs {
    my $compareFields   = shift; # Specfy if the fields should be compared
    my $config1Ref      = shift; # Config 1
    my $config1Name     = shift; # Config 1 name
    my $config2Ref      = shift; # Config 2
    my $config2Name     = shift; # Config 2 name
    my @uniqueSections;

    foreach my $sectionName ( sort keys %$config1Ref ) {
        # check if section exists in config2
        if ( not exists $config2Ref->{$sectionName} ) {
            push(@uniqueSections, $sectionName);
            next;
        }
        my %section1 = %{ $config1Ref->{$sectionName} };
        my %section2 = %{ $config2Ref->{$sectionName} };
        my $firstDifInSection = 1;

        if (not $compareFields) {
            next;
        }

        # Compare the values of each field; print any differences
        foreach my $field ( sort keys %section1 ) {
            if ($section1{$field} ne $section2{$field}) {
                   my $diff1 = compareValues($section1{$field}, $section2{$field});
                   my $diff2 = compareValues($section2{$field}, $section1{$field});

                # If same, skip to next iteration
                if ($diff1 eq "" and $diff2 eq "") {
                    next;
                }

                # If it is the first difference in this section, print section
                # name
                if ($firstDifInSection) {
                    print "$sectionName\n";
                    $firstDifInSection = 0;
                }

                # Print the actual differences
                   print "\t$field\n";
                   if ($diff1 ne "") {
                       print "\t\t$config1Name: ", $diff1, "\n";
                   }
                   if ($diff2 ne "") {
                       print "\t\t$config2Name: ", $diff2, "\n";
                   }
            } # end if
        } # end foreach field
    } # end foreach section

    # If there are unique sections, print them
    if ($#uniqueSections != -1) {
        print "Sections which exist only in $config1Name: ",
            join(", ", @uniqueSections), "\n";
    }
}


########################################################
# Split the path to get just the filename
########################################################
sub getFilenameFromPath {
    my $filename = shift; # the input filename including path
    my @splitName = split(/\//, $filename);
    return $splitName[$#splitName]; # return just the filename, without path
}


########################################################
# Read in the config file section by section.
########################################################
sub readConfig {
    my $filename = shift;
    my %config;
    open CONFIG, "<$filename" or die $!;
    while ( my $line = <CONFIG> ) {
        if ( $line =~ /^\[.*\]$/ ) {
            readSection( $line, \%config );
        }
    }
    close CONFIG;
    return %config;
}


########################################################
# Read in each section of the config file.
########################################################
sub readSection {
    my $line       = shift;
    my $config_ref = shift;
    $line =~ m/\[(.*)\]/;
    my $sectionName = $1;
    while ( my $line = <CONFIG> ) {
        if ( $line =~ /^$/ ) {
            last;
        }
        my @field     = split( /=/, $line );
        my $fieldName = $field[0];
        my $value     = $field[1];
        chomp $value;
        $config_ref->{$sectionName}{$fieldName} = $value;
    }
}
