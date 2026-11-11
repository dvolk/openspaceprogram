               ASTEROID LIGHTCURVE DATA BASE (LCDB)
                                 Revised 2021 August 22



SPECIAL NOTICES
The pds_readme.txt file is no longer distributed. Only the bookmarked PDF version is included.

The lc_notesex (notes, extended) has been renamed to lc_exnotes.

2021 April VERY IMPORTANT
Changes Regarding Phase Slope Parameter G(1), G2
To accommodate the currently adopted absolute magnitude/phase slope parameter H‐G12
(H‐G1,2) systems that replaced the traditional H‐G system, new fields have been added to the
Summary and Details tables and reports. See section 3.1.1.

Changes Regarding Groups/Families
The LCDB has been revised to use a hybrid of the Nesvorny (2015) families and those defined on
the AstDys (2021) web site. As such, all entries in the “Family” column in those reports that
include it, now have a text value that represents a number from either the Nesvorny or AstDys
site family definitions. See section 3.1.2   FAMILY/GROUP MEMBERSHIP, DEFAULT ALBEDOS,
AND TAXONOMIC CLASS.

As a result of the changes above, column mappings for the lc_summary, lc_details, and
lc_diameters reports have changed and there is a new lc_familylookup table. The new listings are
given the appropriate subsections of Section 4.

2020 December
The CLASS column in the Summary and Details table was expanded to 10 characters. This changed
the column mapping for this and all subsequent columns.

2020 March
The structure of the lc_colorindex table was changed to include the R‐I (Cousins) color index after
the V‐I color index and the ATLAS c‐o color index as the last column. The column mapping has
been updated.

2020 February
The structure of the Summary and Details table was changed. The "sparse" and "wide" fields
were replaced by a single "survey" field. The net effect to the column mapping is only that the
survey field starts in the same location as the Sparse field and has the same width.

See Section 5 "HANDLING SURVEY DATA" (updated 2020 February 10)
See Section 6, “NUMBERS OF INTEREST”

New: Section 7, “REFERENCES”. This list the citations for the references mentioned in this file.

The Min/Max Amplitude values in the Summary table are based only on detail lines that have
U  2‐ ratings. If the U code is empty or U  1+, the detail line min/max amplitudes are not
considered.

Floating point numbers are stored as strings in the LCDB. This preserves the original precision of
the data. It is up to the end user to maintain the original precision if/when converting string
representations to real values.

N.B. All lightcurve amplitudes are peak‐to‐peak, not average‐to‐peak.

ASTEROID NAMES
In recent times, asteroid names have included characters that are not easily represented under
extended ASCII encoding. For this reason, the ASCII names are used in the LCDB, i.e., no diacritical
marks are included. For a more accurate listing, the user is referred to the MPC site

       https://www.minorplanetcenter.net/iau/lists/MPNames.html

which gives the current list of numbered/named asteroids. Even this list, however, does not
completely cover all cases. For example,

       (229762) Gǃkúnǁ'hòmdímà

 appears as

       G!kun||'homdima

 in the MPC listing.

PHOTOMETRIC BAND NAMES
To avoid issues with upper/lower case photometric bands, e.g., r vs. R for Sloan r´ vs. Cousins R,
the Sloan band names have be prepended with ‘S’ and made upper case, e.g., r´ => SR. For ATLAS
o and c bands, AO and AC are used. For Gaia GB, GG, and GR (blue, green, red) are used.
1.0.0 INTRODUCTION
The Asteroid Lightcurve Data Base (LCDB) is a set of tables generated from a MySQL database
that includes information directly and indirectly obtained from observations made to determine
the period and/or amplitude of asteroid lightcurves. The information is taken from numerous
journals and other sources.

Its main purpose is to provide a central location for basic information about asteroid rotation
rates and related information that can be used in statistical studies involving a few or many
parameters. Some of the data are obtained directly from the observations while other data are
inferred or calculated based on orbital characteristics, assumed class, etc.

Sections below explain in detail which data are direct and indirectly obtained and, for the latter,
their derivation.

N.B. Even direct data should be confirmed by reference to the original works whenever
possible. Indirect data are provided for information purposes only. They should be used with
great care, if at all, in critical studies.


1.1.0 AUTHOR INFORMATION
These data tables are maintained by Brian D. Warner (Center for Solar System
Studies/MoreData!, Alan Harris (MoreData!), and Petr Pravec (Astronomical Institute, Prague,
Czech Republic).

For basic information on the database or updated versions of the tables, contact:

Brian D. Warner
Center for Solar System Studies / MoreData!
446 Sycamore Ave.
Eaton, CO 80615
brian@MinPlanObs.org

For more theoretical details and discussions, contact

Alan Harris
MoreData!
4603 Orange Knoll Ave.
La Canada, CA 91011 USA
harrisaw@att.net
1.2.0 DISCLAIMER
We have made every attempt to keep the data up‐to‐date and correct. However, we know that
there is the possibility for omissions or errors. Please let us know of any corrections or additions
by sending email to one of the below.

Brian D. Warner brian@MinPlanObs.org
Alan Harris   harrisaw@att.net


2.0.0 DATA FILES
The LCDB release consists of 10 primary files (tables). Also included is the pds_readme.pdf (this
file), which provides detailed information about the LCDB and the 10 tables.


2.0.1 AVAILABILITY OF RAW DATA FILES
The raw databases files are MySQL tables. These are used on the alcdef.org and minorplanet.info
web sites for user‐defined searches.

The MySQL tables are not generally available. However, we can ‐ on a limited basis ‐ provide CSV
(actually, semi‐colon) files generated from the database files that can used to populate local
MySQL files.


2.1.0 DISTRIBUTION FILES
pds_readme.pdf         This file of introductory information.
lc_ambiguous           Lists asteroids with ambiguous periods.
lc_binary              Lists suspected/confirmed binary/multiple asteroids
lc_colorindex          Lists color indexes of asteroids (B‐V, V‐R, V‐I, g‐r, r‐i, B‐R).
lc_details             Lists basic summary table information and the detailed information from
                       individual references. There are one or more lines per asteroid.
lc_diameters           Lists summary H, p_V, D values and the same values plus errors from detail
                       records that reported a diameter.
lc_exnotes             Lists extended notes associated with summary and/or details records.
lc_familylookup        Families and orbital groups used in the LCDB and default albedo, G, and
                       taxonomic class for each one.
lc_npa                 Lists suspected/confirmed asteroids in non‐principal axis rotation (NPAR,
                       or "tumbling").
lc_references          Lists all publications referenced in the LCDB.
lc_spinaxis           Lists asteroids with reported spin axis (poles) and/or shape models.
lc_summary            Lists summary data, one line per asteroid, no references.


2.1.1 SPIN AXIS CATALOGS
As noted above, the lc_spinaxis table stores information about the spin axis properties (ecliptic
coordinates and sidereal periods). A more thorough catalog is maintained by Kryszczynska et al.
at the Poznan Observatory in Poland.

That catalog can be accessed via

       http://vesta.astro.amu.edu.pl/Science/Asteroids/

Josef Durech (Durech et al., 2010) also maintains a list of spin axis solutions, with shape models
and data files. His site is at

       https://astro.troja.mff.cuni.cz/projects/damit/

It should be noted that the favored DAMIT pole may differ from the one in the original reference.
This is usually because Durech and associates did a new analysis with the original, updated,
and/or new data. Their revised result replaced the original instead of indicating a new result
under a different reference.

Because of the complexities of cross‐checking the LCDB vs. DAMIT vs. original result, the LCDB
does not directly include any DAMIT results, i.e., there are no entries with DAMIT being the
author reference. When and if revised results are published in one of the journals, those results
will be included in the LCDB.


2.2.0 BINARY ASTEROID FILES
In addition to the lc_binary table, Petr Pravec (Astronomical Institute, Prague, Czech Republic)
maintains a considerably more detailed set of files:

BINARY_README.TXT         Separate README pertaining the binary asteroid
BINASTD_PUB.TXT           The best estimates of compiled parameters
BINASTE_PUB.TXT           Uncertainties of the estimates in BINASTD_PUB.TXT
BINASTM_PUB.TXT           References and notes for the compiled estimates
BINASTR_PUB.TXT           Information on each of the estimates, e.g., their derivation
These files are not included in the PDS release but are available at
       http://www.asu.cas.cz/~asteroid/binastdata.htm
2.2.1 ABOUT BINARY DATA
The data in the lc_binary table are by no means exhaustive. They are meant to provide a quick
overview of the primary period and amplitude as well as a secondary period and/or orbital period
and, if available, amplitude. Also included, if available, are the depth (magnitude drop) of mutual
events, the Ds/Dp (effective diameter) ratio of satellite to primary, and ADp (semi‐major axis to
primary diameter ratio).

The reader is urged to consult the original journal articles for more complete details.

See the web site run by Wm. Robert Johnston

       http://www.johnstonsarchive.net/astro/asteroidmoons.html


3.0.0 LCDB DATA
The original lightcurve database was a simple text file with a structure that tried to minimize disk
space requirements. That served well for many years. However, the rapidly growing number of
lightcurves being reported as well as the file's format not being able to accommodate some data
prompted a change starting in mid‐2006.

The foremost change was converting to a relational database that included numerous tables and
had SQL search capabilities. This allowed for not only easier maintenance of the database but for
generating reports in a way that are more informative, complete, and consistent in formatting.

The sections below provide the formatting for each field in each table. Of particular importance
is to note the ‐maximum‐ precision of floating‐point numbers is not always the actual precision
of the reported value. In critical studies, it is important for end‐users to keep the original precision
of the values.


3.1.0 DIRECT DATA
Data that are obtained directly from photometric observations includes

   1. Rotation period (usually synodic).
   2. Amplitude.
   3. Absolute magnitude, H, and phase slope parameter, G, when determined by using
      reduced magnitude versus phase angle data.
   4. Binarity due to mutual events, i.e., occultations and eclipses. In such cases, the rotation
      period of the primary and orbital period of the satellite and the amplitude of the primary
      lightcurve are the usual direct results. The size ratio can be computed from the depth of
      the events. For more details on binary lightcurve analysis, see Pravec et al. (2006).
   5. Color indices.
   6. Diameter if based on stellar occultation or adaptive optics/radar. Radar diameters can
      also be considered indirect depending on how the diameter was determined.
   7. Taxonomic class.


3.1.1 THE H‐G, H‐G12, and H‐G1,G2 SYSTEMS
To predict the magnitude of an asteroid and how it changes with phase angle, the H‐G system
(Bowell et al., 1989) was adopted by the IAU in 1985. A new system involving three parameters
(Muinonen et al., 2010) was adopted by the IAU, though the H‐G system still remains in wide use.

The H‐G12 system uses only the first term of the two under the H‐ G1,G2 system and is most
often used when there are limited data for finding a phase curve. The H‐G1,G2 system uses two
parameters that more effectively describe the asteroid’s brightness at large and small phase
angles. It has also been shown to be an effective tool for taxonomic classification (Shevchenko et
al, 2016; Mahlke et al., 2021).

A strong discontinuity exists around G1 = 0.2 for the H‐G12 and H‐G1,G2 systems. See the
Muinonen et al. (2010) paper for a discussion of this issue.

The large majority of G values given in the LCDB are under the H‐G system. However, some
surveys that produced large number of rotation periods also found G1 under H‐G12 as part of
their data reduction. To accommodate both systems, the Summary and Details tables include a
“G” and “G2” field.

In both tables, if the “GSource” field has no or a value other than ‘G’, the “G” data field is on the
H‐G system. If the “GSource” field has ‘G’, then the value in the “G” field is G1 on the H‐G12 or
H‐G1,G2 system. If the “G2” field has a value, then the “G” field is G1 on the H‐G1,G2 system. The
Diameters report (see section 4.5.0 LC_DIAMETERS”) also accommodates the two G fields.


3.1.2 FAMILY/GROUP MEMBERSHIP, DEFAULT ALBEDOS, AND TAXONOMIC CLASS
Until 2021 April, the LCDB defined 33 “families,” which were actually groups based on orbital
parameters using osculating elements rather than membership in a collisional family. These were
replaced by adopting the dynamical families defined under Nesvorny (2015) and Nesvorny et al.
(2015) Nesvorny refers to both references from here on) and AstDys (2021) web site (see the
numerous references available there). The most important element of the revision is that family
membership is based on proper elements.

More so, Nesvorny used SDSS colors and WISE (Mainzer et al., 2019) albedos along with a
parameter Cj to isolate objects that were more likely than not true family members (from the
same parent) and those just occupying the orbital space of a family, what they called dynamic
interlopers. For example, their list for the Hungaria family includes 2965 objects but 60 of those
were flagged by the Cj parameter to by dynamic interlopers.

In the LCDB, any object not within one of the defined families is given an LCDB‐defined family
identification number (FIN; see Nesvorny for a discussion about the need and use of FINs) in the
range 9000‐9999.

Whether a true family member or a group member, a default albedo, taxonomic class, and G (on
the H‐G system) are assigned to an object when added to the LCDB, except in those cases when
one or more of the actual values are known. These three values can have a complex relationship
when the class and albedo are not directly obtained.

Nesvorny assigned default albedos and taxonomic class (usually Bus‐DeMeo but sometimes
Tholen) for most of the families in their list. As previously mentioned, the albedos are based on
WISE observations, i.e., their H and the measured diameter were used to derive the albedo, while
the taxonomic class was taken from the literature. When no actual values are available, the
defaults are used. However, Nesvorny did not provide a default value for G. This value is assigned
based on the adopted albedo as given in Table 3.1.2.1.

                    Taxonomic Classes             Albedo                G
                    C, G, B, F, P, T, D        0.057 ± 0.02        0.12 ± 0.08
                    M                           0.16 ± 0.04        0.20 ± 0.07
                    S, Q                        0.20 ± 0.07        0.24 ± 0.11
                    E, V, R                     0.46 ± 0.06        0.43 ± 0.08
                  Table 3.1.2.1. Inter‐relation of default taxonomic class,
                  albedo, and G (H‐G) from Warner et al. (2009).

While the basic classifications in Table 3.1.2.1 are those from Tholen (1984), the values for each
class were based on known values within each “complex.” For example, the values for S were
derived from objects classified as “S”, “Sr”, “Sq”, etc. under the Bus‐DeMeo taxonomy. Beyond
this, Table 3.1.2.1 (originally in Warner et al., 2009) is used to assign default values when at least
one of the three parameters is known. These defaults were based on objects with known values
for all three parameters at the time.

It should be noted that if there is a measured H that is considered reliable, it will be used in lieu
of value in the MPCORB file. Likewise, if there is a reliable diameter, e.g., from a spacecraft or
occultation, it will be used in lieu of a default value or that measured by an IR survey. The
associated values for H and albedo adjusted accordingly. In other words, whenever possible, a
measured value takes precedence over its corresponding default value.

The Nesvorny family numbers were used for the initial assignments, including for those in AstDys
families common to Nesvorny. However, there were 60 families in the AstDys list that were not
in Nesvorny. These were assigned custom family numbers in the range 2000‐2999.
Table 3.1.2.2 gives a full listing of the 202 family or orbital group assignments and default values
used in the 2021 April release of the LCDB.
The Number column is the family number that appears in the LCDB text reports. Nesvorny
numbers range from 001‐999. Family numbers 2001‐2060 are dynamical families that appear
only in the list on the AstDys (2021) web site. Numbers > 9100 are broadly‐defined orbital groups
for those objects that could not be tied to a dynamic family. e.g., NEA (9101), Hungaria (9102),
Mars‐crosser (9103), etc. The family name is appended with an asterisk.
The Parent is the MPC‐assigned number of the parent (often largest) body of a family. The Name
column gives the name of the parent body. The Albedo column value is usually that of the parent
body as inferred by WISE (Mainzer et al., 2019) or, for a small number of families, the average of
several albedos of family members, again using albedos inferred from WISE data.
If there is an asterisk after a class, it is assumed based on orbital location. Otherwise, it is the
measured taxonomic class of the parent body. Count is the number of dynamic family members
taken from the Nesvorny or AstDys lists.

Number Parent Name                          Count      Albedo       Err       G        Err    Class
  001    153 Hilda                            409       0.038      0.007      0.12     0.08    P
  002   1911 Schubart                         352       0.039      0.013      0.12     0.08    P
  003    434 Hungaria                        2965       0.38       0.1        0.43     0.08    E
  004    624 Hector                            12       0.107      0.011      0.24     0.11    C
  005   3548 Eurybates                        218       0.052      0.007      0.12     0.08    C*
  006   9799 1996 RJ                            7       0.06       0.012      0.12     0.08    C*
  008 20961 Arkesilaos                         37       0.057      0.02       0.12     0.08    C*
  009   4709 Ennomos                           30       0.077      0.009      0.12     0.08    D
  010 247341 2001 UV209                        13       0.088      0.023      0.12     0.08    C*
  401      4 Vesta                          15252       0.355      0.099      0.43     0.08    V
  402      8 Flora                          13786       0.24       0.01       0.24     0.11    S
  403    298 Baptistina                      2500       0.154      0.02       0.24     0.11    S*
  404     20 Massalia                        6424       0.249      0.07       0.24     0.11    S
  405     44 Nysa-Polana                    19073       0.48       0.02       0.43     0.08    E
  406    163 Erigone                         1776       0.055      0.013      0.12     0.08    C
  407    302 Clarissa                         179       0.054      0.02       0.12     0.08    F
  408    752 Sulamitis                        303       0.055      0.01       0.12     0.08    P
  409   1892 Lucienne                         142       0.25       0.03       0.24     0.11    S
  410     27 Euterpe                          474       0.22       0.01       0.24     0.11    S
  411   1270 Datura                             6       0.29       0.01       0.24     0.11    S
  412 21509 Lucascavin                          3       0.24       0.02       0.24     0.11    S*
  413     84 Klio                             330       0.053      0.002      0.12     0.08    C
  414    623 Chimaera                         108       0.035      0.004      0.12     0.08    Xc
  415    313 Chaldaea                         132       0.05       0.01       0.12     0.08    C
  416    329 Svea                              48       0.046      0.007      0.12     0.08    C
  417 108138 2001 GB11                          9       0.2        0.07       0.24     0.11    S*
  501      3 Juno                            1684       0.253      0.055      0.24     0.11    S
  502     15 Eunomia                         5670       0.26       0.083      0.24     0.11    S
Number Parent Name            Count   Albedo   Err     G      Err    Class
 504     128   Nemesis         1302   0.051    0.002   0.12   0.08   C
 505     145   Adeona          2236   0.062    0.01    0.12   0.08   C
 506     170   Maria           2940   0.261    0.084   0.24   0.11   S
 507     363   Padua           1087   0.06     0.01    0.12   0.08   X
 508     396   Aeolia           296   0.106    0.028   0.24   0.11   X
 509     410   Chloris          424   0.05     0.01    0.12   0.08   C
 510     569   Misa             702   0.058    0.016   0.12   0.08   C
 511     606   Brangane         195   0.121    0.028   0.24   0.11   K
 512     668   Dora            1259   0.058    0.014   0.12   0.08   C
 513     808   Merxia          1215   0.248    0.055   0.24   0.11   S
 514     847   Agnia           2125   0.242    0.056   0.24   0.11   S
 515    1128   Astrid           489   0.052    0.014   0.12   0.08   C
 516    1272   Gefion          2547   0.25     0.01    0.24   0.11   C*
 517    3815   Konig            354   0.051    0.014   0.12   0.08   C*
 518    1644   Rafita          1295   0.14     0.03    0.24   0.11   S
 519    1726   Hoffmeister     1819   0.048    0.013   0.12   0.08   c*
 520    4652   Iannini          150   0.26     0.05    0.24   0.11   S
 521    7353   Kazuya            44   0.2      0.05    0.24   0.11   S*
 522     173   Ino              463   0.07     0.02    0.12   0.08   X
 523   14627   Emilkowalski       4   0.2      0.02    0.24   0.11   D
 524   16598   Brugmansia         3   0.1      0.05    0.12   0.08   C*
 525    2384   Schulhof           6   0.25     0.04    0.24   0.11   S*
 526   53546   2000 BY6          58   0.057    0.02    0.12   0.08   C*
 527    5438   5438 Lorre         2   0.05     0.02    0.12   0.08   C
 528    2782   Leonidas         135   0.057    0.02    0.12   0.08   C*
 529     144   Vibilia          180   0.05     0.01    0.12   0.08   C
 530     322   Phaeo            146   0.08     0.02    0.12   0.08   C
 531    2262   Mitidika         653   0.21     0.04    0.24   0.11   S*
 532    2085   Henan           1872   0.19     0.08    0.24   0.11   L
 533    1668   Hanna            280   0.04     0.01    0.12   0.08   c*
 534    3811   Karma            124   0.05     0.02    0.12   0.08   C*
 535    2732   Witt            1816   0.3      0.03    0.24   0.11   S
 536    2344   Xizang           275   0.09     0.01    0.12   0.08   C
 537     729   Watsonia          99   0.13     0.02    0.24   0.11   L
 538    3152   Jones             22   0.05     0.02    0.12   0.08   C
 539     369   Aeria            272   0.16     0.02    0.24   0.11   M
 540      89   Julia             33   0.18     0.02    0.24   0.11   K
 541    1484   Postrema         108   0.038    0.014   0.12   0.08   C*
 601      10   Hygiea          4854   0.073    0.022   0.12   0.08   C
 602      24   Themis          4782   0.07     0.019   0.12   0.08   B
 603      87   Sylvia           255   0.042    0.01    0.12   0.08   X
 604     137   Meliboea         444   0.05     0.01    0.12   0.08   C
 605     158   Koronis         5949   0.24     0.06    0.24   0.11   S
 606     221   Eos             9789   0.157    0.05    0.24   0.11   C
 607     283   Emma              76   0.049    0.013   0.12   0.08   P
 608     293   Brasilia         579   0.174    0.042   0.24   0.11   X
 609     490   Veritas         1294   0.06     0.01    0.12   0.08   C
Number Parent Name         Count   Albedo   Err     G      Err    Class
 610    832 Karin            541   0.22     0.04    0.24   0.11   S
 611    845 Naema            301   0.065    0.014   0.12   0.08   C
 612   1400 Tirela          1395   0.22     0.01    0.24   0.11   S*
 613   3556 Lixiaohua        756   0.047    0.01    0.12   0.08   T
 614   9506 Telramund        468   0.057    0.02    0.12   0.08   C*
 615 18405 1993 FY12         104   0.184    0.042   0.24   0.11   S*
 616    627 Charis           808   0.09     0.04    0.12   0.08   X
 617    778 Theobalda        376   0.06     0.02    0.12   0.08   F
 618   1189 Terentia          79   0.049    0.01    0.12   0.08   C
 619 10811 Lau                56   0.26     0.02    0.24   0.11   S*
 620    656 Beagle           148   0.08     0.02    0.12   0.08   C*
 621    158 Koronis(2)       246   0.24     0.06    0.24   0.11   S
 622     81 Terpsichore      138   0.05     0.01    0.12   0.08   C
 623    709 Fringilla        134   0.04     0.01    0.12   0.08   P
 624   5567 Durisen           27   0.058    0.015   0.12   0.08   C*
 625   5614 Yakovlev          67   0.06     0.01    0.12   0.08   C*
 626   7481 San Marcello     144   0.17     0.07    0.24   0.11   X
 627 15454 1998 YB3           38   0.045    0.01    0.12   0.08   C*
 628 15477 1999 CG1          248   0.057    0.02    0.12   0.08   C*
 629 36256 1999 XT17          58   0.19     0.04    0.24   0.11   A
 630     96 Aegle             99   0.051    0.01    0.12   0.08   T
 631    375 Ursula          1466   0.062    0.015   0.12   0.08   X
 632    618 Elfriede          63   0.057    0.02    0.12   0.08   C
 633    918 Itha              54   0.19     0.02    0.24   0.11   C*
 634   3438 Inarradas         38   0.06     0.02    0.12   0.08   C
 635   7468 Anfimov           58   0.257    0.04    0.24   0.11   S*
 636   1332 Marconia          34   0.051    0.01    0.12   0.08   L
 637 106302 2000 UJ87         64   0.044    0.01    0.12   0.08   C*
 638    589 Croatia           93   0.049    0.01    0.12   0.08   C
 639    926 Imhilde           43   0.055    0.01    0.12   0.08   C
 641    816 Juliana           76   0.037    0.02    0.12   0.08   C*
 701     25 Phocaea         1989   0.253    0.117   0.24   0.11   S
 801      2 Pallas           128   0.14     0.02    0.24   0.11   B
 802    148 Gallia           182   0.21     0.03    0.24   0.11   S
 803    480 Hansa           1094   0.286    0.068   0.24   0.11   S
 804    686 Gersuind         415   0.1      0.02    0.12   0.08   S
 805    945 Barcelona        306   0.3      0.1     0.24   0.11   S
 806   1222 Tina              96   0.17     0.04    0.24   0.11   X
 807   4203 Brucato          342   0.07     0.02    0.12   0.08   SC*
 901     31 Euphrosyne      2035   0.061    0.015   0.12   0.08   C
 902    702 Alauda          1294   0.06     0.01    0.12   0.08   B
 903    909 Ulla              26   0.036    0.01    0.12   0.08   P
 904   1303 Luthera          163   0.053    0.01    0.12   0.08   C*
 905    780 Armenia           40   0.04     0.01    0.12   0.08   M
Number Parent Name             Count   Albedo   Err      G      Err    Class
2001       5   Astraea          6169   0.2694   0.0762   0.24   0.11   S
2002      93   Minerva          2428   0.2765   0.096    0.24   0.11   C
2003     110   Lydia             898   0.0731   0.0198   0.12   0.08   X
2004     135   Hertha          15983   0.179    0.032    0.24   0.11   X
2005     159   Aemilia            62   0.064    0.014    0.12   0.08   C
2006     179   Klytaemnestra     513   0.198    0.011    0.24   0.11   S
2007     194   Prokne            379   0.145    0.037    0.2    0.07   C
2008     260   Huberta            26   0.044    0.01     0.12   0.08   P
2009     883   Matterania        169   0.206    0.019    0.24   0.11   S
2010     895   Helio              50   0.071    0.018    0.12   0.08   B
2011    1040   Klumpkea         1815   0.2039   0.1      0.24   0.11   C*
2012    1101   Clematis           17   0.127    0.019    0.2    0.07   C*
2013    1118   Hanskya           116   0.056    0.003    0.12   0.08   C*
2014    1298   Nocturna          186   0.054    0.01     0.12   0.08   X
2015    1338   Duponta           133   0.251    0.04     0.24   0.11   S*
2016    1547   Nele              344   0.355    0.064    0.43   0.08   S
2017    1658   Innes             775   0.0554   0.013    0.12   0.08   A
2018    2076   Levin            1534   0.2015   0.07     0.24   0.11   S
2019    3025   Higson             17   0.074    0.016    0.12   0.08   C
2020    3330   Gantrisch        1178   0.0471   0.012    0.12   0.08   T
2021    3460   Ashkova            59   0.066    0.015    0.12   0.08   C
2022    3561   Devine             19   0.092    0.014    0.12   0.08   C*
2023    3827   Zdenekhorsky     1050   0.074    0.02     0.12   0.08   C*
2024    5026   Martes            481   0.0554   0.013    0.12   0.08   C
2025    5651   Traversa           56   0.036    0.001    0.12   0.08   D
2026    5931   Zhvanetskij        23   0.053    0.007    0.12   0.08   C*
2027    6124   Mecklenburg        78   0.067    0.011    0.12   0.08   C*
2028    6355   Univermoscow       13   0.065    0.02     0.12   0.08   Xt
2029    6769   Brokoff            58   0.068    0.01     0.12   0.08   C*
2030    7605   Cindygraber        19   0.029    0.001    0.12   0.08   C*
2031    7744   1986 QA1           98   0.229    0.031    0.24   0.11   S
2032    8060   Anius              31   0.059    0.013    0.12   0.08   C*
2033    8737   Takehiro           57   0.085    0.012    0.12   0.08   C*
2034   10369   Sinden             24   0.062    0.01     0.12   0.08   SC*
2035   10654   Bontekoe           13   0.08     0.015    0.12   0.08   C*
2036   10955   Harig             918   0.057    0.02     0.12   0.08   C*
2037   11097   1994 UD1           33   0.089    0.011    0.12   0.08   C*
2038   11882   1990 RA3           87   0.372    0.191    0.3    0.1    Xe*
2039   12739   1992 DY7          298   0.258    0.04     0.24   0.11   S*
2040   13314   1998 RH71         241   0.056    0.003    0.12   0.08   C
2041   14916   1933 VV7           17   0.133    0.027    0.2    0.07   S*
2042   16286   4057 P-L           94   0.249    0.07     0.24   0.11   S*
2043   17392   1981 EY40          96   0.1      0.05     0.12   0.08   SC*
2044   17492   Hippasos            7   0.066    0.008    0.12   0.08   C*
2045   18466   1995 SU37         257   0.057    0.02     0.12   0.08   C*
2046   21344   1997 EM            75   0.143    0.04     0.2    0.07   S*
2047   21885   1999 UY27          61   0.039    0.006    0.12   0.08   C*
 2048 22805 2000 AA169                          20      0.086      0.019      0.12     0.08    C*
 2049 23255 2000 YD17                           12      0.188      0.021      0.2      0.07    S*
 2050 29841 1999 FO14                           65      0.1        0.05       0.12     0.08    SC*
 2051 31811 1999 NA41                          144      0.063      0.014      0.12     0.08    C*
 2052 32418 2000 RD33                           81      0.057      0.02       0.12     0.08    C*
 2053 40134 Marsili                             16      0.199      0.082      0.24     0.11    S*
 2054 43176 1999 XM196                          75      0.057      0.02       0.12     0.08    C*
 2055 45637 2000 EW12                           20      0.057      0.02       0.12     0.08    C*
 2056 58892 1998 HP148                          20      0.048      0.009      0.12     0.08    C*
 2057 69559 1997 UG5                            17      0.044      0.007      0.12     0.08    C
 2058 116763 2004 EW7                           24      0.148      0.129      0.2      0.07    S*
 2059 222861 2002 EZ134                         11      0.087      0.019      0.12     0.08    C*
 2060 291316 2006 BE167                          9      0.07       0.014      0.12     0.08    C*
Table 3.1.2.2. The dynamic families and orbital groups used in the LCDB. The Albedo and G
columns are the default values used when actual values are not available. A class followed by * is
the default based on orbital location using osculating elements.

Table 3.1.2.3 gives the osculating orbital elements ranges for the broad groups with family
numbers > 9000. For each core element (a, e, i), the two columns represent the minimum and
maximum values. Q is aphelion distance and q is perihelion distance.

Family          Orbital Elements Description               Class    Albedo        G        Note
9101     q < 1.3                                            S       0.20         0.24
9102     1.78 < a < 2.0 e < 0.18 16° < i < 34°              ES      0.3          0.30      1
9103     1.3 < q < 1.668 Q < 5                              S       0.20         0.24      2
9104     a < 2.6                                            S       0.20         0.24
9105     2.6 < a 2.7                                        SC      0.10         0.15
9106     2.7 < a < 5.0                                      C       0.057        0.12
9107     3.7 < a < 4.2 0.07 < e < 0.3 i < 20°               P       0.057        0.12
9108     5.5 < a < 30                                       C       0.057        0.12
9109     a > 30                                             C       0.1          0.2
9201     elements similar to Mars                           S       0.20         0.24      3
9202     5.0 < a < 5.4                                      C       0.057        0.12
9203     elements similar to Saturn                         C       0.057        0.12      4
9204     elements similar to Uranus                         C       0.057        0.12      4
9205     elements similar to Neptune                        C       0.057        0.12
9301     Q > 5.0                                            C       0.057        0.12      5
9302     exhibits coma and/or tail                          C       0.057        0.12
9401‐9405
Comets in the orbital space of respectively, NEA,
Main belt inner/middle/outer, and Centaurs                  C       0.057        0.12
9501     e  1.0                                            C       0.057        0.12
9502     Planetary satellite                                C       0.057        0.12
Table 3.1.2.3. The orbital parameters for the broad orbital groups used to classify objects that are
not family members under Nesvorny or AstDys.
Notes for Table 3.1.2.3

1    pV = 0.3 is a compromise value when no taxonomic information is available since the
     Hungarias are both a family (common parent, E/X class, pV = 0.4) and group (similar orbits,
     S class, pV = 0.20).
2    Generally, there are no inclination limits on Mars‐crossers.
3    The default pV = 0.20 ± 0.07 for S‐type objects was derived from the geometric mean of all
     S‐type objects in the LCDB with known albedos (usually SIMPS).
4    None known and not likely to be found due to perturbations by the other gas giants.
5    Barring any other classification that meets the Q > 5.0 requirement, the orbit is considered
     “comet like.”


3.1.2.1 CAVE USOR – USER BEWARE
We wish to make it clear as possible that an object being in the same orbital space defined for a
family does not necessarily make it a member of that family but, instead, that it could be a
dynamic interloper. The Hungaria family is just one example where there are numerous
interlopers.

The true determination of family membership is possible only when its proper elements and
taxonomic spectrum (not just broad taxonomic class) closely match those of a known family
parent. The assignments in the LCDB are meant to be good starting points and may be useful in
many cases but they are not the final word for critical studies.

A Case in Point
For some time, 93 Minerva was considered the main (namesake) body for the Minerva family.
Spectroscopic observations showed that it was actually an interloper among what is now called
the Gefion family (after 1272 Gefion). What distinguishes the Gefion members is their higher
than usual albedo for outer main‐belt objects (pV ~ 0.25) instead of the more typical pV ~ 0.06.

The AstDys families list has only the Minerva group, which has relatively few members common
to the Nesvorny Gefion family (516). The Nesvorny Gefion family is used in the LCDB. Members
of the AstDys list not in the Nesvorny list were placed in the catch‐all outer main‐belt group
(9106). However, they were not all set to use the defaults of class = C*, G = 0.12 ± 0.08, and
albedo = 0.057.

Where available, WISE, AKARI, and/or SIMPS albedos for an object were averaged and that value
was assigned with the ‘L’ (details record) flag. Based on Table 3.1.2.1, objects with albedos < 0.12
were set to type “C*.” Type “SC*” was assigned to objects within 0.12  pV  0.18 but the
averaged albedo was used in lieu of the default of pV = 0.1. Objects with pV > 0.18 were assigned
the “S*” class. These somewhat arbitrary assignments should reinforce that the albedos in the
LCDB are not the final word.
Another Case in Point
While there are some specific families within the common orbital space for the near‐Earth
asteroids, Hildas, Jupiter trojans, Centaurs, and TNO/KBO objects, in general these groups are
treated without distinction between family and group members since there is no single parent
body for each group.

For example, when searching the MPCORB file using the osculating elements limits for group
9107 (Hilda space in Table 3.1.2.3), a total of 4615 objects were found. However, looking deeper,
409 of those objects were in families or subfamilies defined by Nesvorny and/or AstDys other
than the Hildas (e.g., Nesvorny 002, Schubart; AstDys 2022, Devine; AstDys 2027, Mecklenburg).

The 2021 April snapshot of the LCDB summary table found 1484 objects, regardless of U rating,
within the orbital space in Table 3 (the previous method of assigning membership to the Hildas).
Of those, 258 were assigned to “outsider” families such as those in the previous paragraph.

These are just some of the many examples of the indefinite definition of asteroid families and
the possibility for numerous interlopers with “outlying” values in a presumed family. If trying to
do critical studies based on true family membership, the LCDB should be considered a starting
point but not the final destination.

“Cave Usor!”


3.1.2 SYNODIC VERSUS SIDEREAL PERIOD
The synodic period depends on viewing aspect and the rate of motion of the asteroid across the
sky. An expression for the magnitude of the expected difference between the sidereal period and
synodic period based on the phase angle bisector (PAB; see Harris et al., 1984) is

        DeltaP = [d(PAB)/dt] * P^2

where    DeltaP         difference between synodic and sidereal periods, in units of the rotation
                        period (usually hours).

         [d(PAB)/dt]    angular rate of change in the Phase Angle Bisector in inverse units of the
                        rotation period, e.g., in units of cycles/hour

         P              the synodic rotation period of the asteroid in the same units of time e.g.,
                        hours/cycle

For example, assume an asteroid with a rotation period of 8 hours observed when the PAB is
changing by 0.05 deg/day (typical for a main‐belt asteroid at opposition). The sidereal‐synodic
difference is

        DeltaP = [0.05 / 360.0 / 24.0] *   (8 ^ 2)
               = 0.00037 hr.
The synodic‐sidereal difference can be either positive or negative, and can exceed the value given
by this expression for near‐polar aspects, but the expression gives a reasonable estimate of the
magnitude of the expected difference.

In most cases, the period given in the summary and details lines is synodic and not sidereal. An
'S' flag (see notes below) indicates that the period is sidereal. There are many entries in both
tables that do not carry the 'S' flag when they should. This is part of the legacy nature of the data
after converting the files to the new data base, i.e., the old format did not allow for indicating
the period was one type or another. We hope to update these and other legacy values that now
have qualifying flags in future versions.

For most studies, the difference between sidereal and synodic period is not significant.


3.1.2 INDIRECT DATA
Indirect data are those obtained by calculation and/or assumption.

Diameter, H, and albedo (p_V, p_R, etc.)

The relationship between H, diameter, and albedo is:

       D = (1329km) * 10.0^(‐0.2*H) / sqrt(albedo)
       or
       logD (km) = 3.1235 ‐ 0.2H ‐ (0.5 * log(albedo)).

The value of H is usually known, though not always accurately, based on photometric
observations. If direct data are available for D and/or p_V, then the above relationships can be
used to derive a missing quantity.

Data from the SIMPS study (Tedesco et al., 2004). NASA Planetary Data System) are used when
available and no overriding data are available. If a newer value of H than that used by SIMPS is
available, the diameter and albedo are re‐computed based on Harris and Harris (1997).

If the diameter was based on an assumed albedo and H is revised, the albedo is held constant
and the diameter re‐computed using the above formulae.

If the diameter was determined by radar, resolved imagery, etc. and a new H is available, the
diameter is held constant and the albedo is re‐computed.

We acknowledge that several, newer IR surveys (e.g., WISE, AKARI, and Spitzer) have reported
diameters. It was an impossible task to weigh the individual results and adopt one for the
summary record. Therefore, the SIMPS diameters are still used. However, the lc_details and
lc_diameters tables both list all included reported diameters. These allow the user to decide for
himself which diameter is the "true" value.
Color Index applied to H

Color index is not generally assumed or entered into the LCDB. However, sometimes the value of
H was found in a photometric band other than V, e.g., Cousins R. In that case, and if the value is
used to override the H given by SIMPS or the MPCORB table (Minor Planet Center) in the
summary record, H is transformed to the V band. When the color index is not directly available,
these values are used to transform the measured H value:

       V‐R: 0.45      B‐V: 0.80       V‐r´: 0.23

When such a transform is used in the summary record, whether or not based on an assumed
value, the H value has the 'T' (transformed) flag.


3.2.0 ORPHAN RECORDS
The full summary (MySQL) table contains almost 300,000 "orphaned" records. These are where
no lightcurve observations have been reported but other data have been stored in the details
and/or other tables in the LCDB, e.g., diameters, color index, taxonomic class, etc. These records,
and any details records associated with the object, are not considered when creating the
lc_summary and lc_details tables.

However, the orphan flag ('O') is ignored when generating the other tables so that the non‐
lightcurve data can still be made available.

See also section 6.0.1 ORPHAN RECORDS


4.0.0 LCDB FILE DESCRIPTIONS
The following sections describe the specific tables that are part of the LCDB release. A column
map is provided for each table that shows the type and format of each field.

N.B.   The column maps in the pds_readme.pdf included with a PDS distribution take
       precedence over the column maps in this document because of PDS requirements

Sub‐sections describe the meaning of the flags that qualify various fields in each table.
4.1.0 LC_SUMMMARY AND LC_DETAILS TABLES
These are the primary tables in the LCDB release. They show both direct and indirect data, the
most important being the direct data of lightcurve period and amplitude, along with our
assessment of the quality of the period solution.

The latter is expressed by the U code, which is described in detail below.

The lc_summary table uses one line per object, which includes the full summary listing for the
asteroid. This line represents our best determination of the primary information for the given
object based on the data in the details table.

For example, where several periods are available, the summary line gives the one that we
consider to be the most likely correct. Sometimes that value may be an average of the available
values.

In the lc_details table, each line includes core summary information followed by the details
record data. There can be multiple lines per asteroid. Each line includes, when available, not only
the reported value but the reported error. Also included is the "Short Reference" that can be
used to find the full citation for the original publication in the lc_references table.


4.1.1 MULTIPLE DETAILS TABLE ENTRIES
In some cases, there is more than one detail line under a given asteroid with the same publication
reference. This is deliberate in order to allow statistical studies of lightcurve amplitude versus
phase versus class (albedo).

For example, if a single publication reports the lightcurve behavior for an asteroid where the
synodic period and/or amplitude of the curve changed significantly during the course of the
observations, the lc_details table will include an appropriate number of entries. Those entries
will "split out" the results so that the period and/or amplitude can be tied to a specific (though
maybe only approximate) set of PAB or Phase values. A good example would be a paper that
reports on observations of an NEA asteroid that occurred over several weeks and the amplitude
of the curve ranged from 1.1 to 0.3 magnitudes over the course of the observations.

In most cases, splitting the results into distinct sets was not difficult, e.g., the asteroid was
observed on one night at one‐week intervals. In some cases, the split was not so distinct. In this
case, compromises were made in order to avoid having an excess of multiple entries while still
retaining sufficient resolution of the variations versus time.

A variation on the above is if the author(s) forced the data from several blocks of dates to fit a
fixed period solution. Here, the period will be the same for all entries, though the amplitude may
change. In this case, the period is left blank for the second and subsequent lines. The U code is
assigned for each lightcurve based on the presumption that the fixed period is correct, i.e., it is
based on the quality of the fit of the data to the presumed period. The main point of interest is
the amplitude for the reasons given above. Other information that was derived based on the
given block of data, e.g., a value for H, G, or a color index, will be included within the details
record as well so that it's clear which block of data was used to derive the given values.


4.1.2 U (QUALITY) CODE
The U code provides our assessment of the quality of the period solution, not necessarily of the
data per se. The uniqueness of the solution, while an important factor, is not the sole
consideration in making an assessment. The quality of the data is sometimes used as a tie‐breaker
when deciding between two half‐steps, e.g., between 2+ and 3‐.

Depending on the specifics for a given asteroid, a good period solution can be obtained by using
a large amount of lesser quality data about as well as using less data that are of higher quality.
Many factors come into play making the assessment. The table below gives the general outline
of the criteria used, going from highest to lowest rating.

   3       The lightcurve is completely unambiguous in terms of period, i.e., there are no cycle
           ambiguities or possible solutions with single, triple, or other number of extrema. The
           coverage of the entire rotation phase is to the degree than any remaining small gaps
           can be confidently interpolated.

   3–      A unique period determination, but possibly some moderate gaps in coverage,
           enough so that interpolation of the entire curve is not certain, but not enough to allow
           any other solution.

   2+      It is unlikely but not impossible that the period is in error due to cycle counts or
           alternate numbers of extrema per cycle, and no more than moderate gaps in coverage
           (as in U = 3‐). Another case is if there are indications that a second period (e.g., due to
           a satellite) might have been overlooked. These can manifest themselves as one or two
           nights where the data showed an unexpected attenuation compared to the rest of the
           lightcurve.

   2       Result based on less than full coverage, so that the period may be wrong by 30 percent
           or so or to note results where an ambiguity exists as to the number of extrema per
           cycle or the number of elapsed cycles between lightcurves. Hence the result may be
           wrong by an integer ratio.

   2–      Period and total amplitude not firmly established. For example, a single night
           coverage of about half a cycle including a maximum and minimum, but not enough to
           derive an accurate period. This is the minimum reliability code that we consider
           acceptable for statistical analysis.
   1+      Similar to U = 2‐, but with less amplitude so that it is not absolutely certain that the
           variations are true rotational variation and not due to noise, etc.

   1       May be completely wrong. What is interpreted as rotational variation may be just
           noise, calibration error, etc.

   1–      Probably wrong. A lightcurve that may be completely wrong (as in U = 1) but, in
           addition, the claimed period is very unlikely, e.g., a large object with a claim of
           P < 2 h.

   0       Result later proven to be incorrect. This appears only in detail table entries, not the
           summary table.

A possible exception occurs when none of the detail records have U > 2+ but the summary line
may have U = 3– or 3. An example would be if one data set in the details table leads to a clearly
unique period but has too sparse or too large a gap in coverage to warrant a 3 rating, but another
data set densely covers the lightcurve at or very close to the uniquely determined period but the
period based on that data set is not necessarily unique. Here, the collective result would rate in
the 3 range, but the individual sets would still be in the 2 range.

It is important to keep in mind that U = 0 does not necessarily mean that the data for a given
lightcurve are of low quality. The only interpretation that should be inferred is that the reported
solution has been determined, perhaps from subsequent data, to be incorrect so that not even
the loose constraints of U = 1 or U = 2 can be used. For the most part, U = 0 will be used very
sparingly and the previous U rating (unless 3) will be retained to avoid the false impression that
the data are of limited or no use.

N.B.    Until the intermediate release in 2008 November, the LCDB also used a value of '4' for the
        U code, which indicated that a pole solution had been reported. This is no longer the case
        since, in the past, there have been cases where a 4 was assigned because there was a pole
        solution given but the best available period solution was no better than 2.

        The period solution quality is now independent of any pole solution. A separate "Pole"
        flag in the summary and details tables is used to indicate that a pole solution has been
        reported. The lc_spinaxis table includes more details and its own quality code assignment.

Assignment of the refined ratings using a + or – is a work in progress as we catch up with almost
30 years of data entry. Therefore, not all U code ratings will match what we would give under
current rules and are subject to change.

Some details lines, and even some summary, may not contain a U code rating. This is deliberate
and can be for several reasons.

  1     The available data do not include a lightcurve, therefore, it is not possible to give a rating
        to the curve. In some cases, where the results are reported by observers whose standard
       of work is known to be of sufficient quality, we may assign an interim U code, usually 2,
       until a lightcurve or the data are available.

  2    In the case where several results are published for a given object in the same reference,
       we will assign a U code rating for the "best" available data and include only new
       information for that given Details record, e.g., see section 4.1.1, “Multiple Details Table
       Entries.”

  3    When the available data do not reasonably define a period or even constrain a range in
       which the period lies. In addition, the data may not provide any reasonable indication of
       the amplitude. The details table entry will have only the reference to the work; the period,
       amplitude, and U code rating will use the default “no data” entries.

The summary line may also have no period and/or amplitude as well as no U code. This occurs
when none of the detail records, even if they have some or all of the information, is deemed
insufficiently reliable to put in the summary line. This is done to show that there are data available
for the object but that they may be of limited use.
4.1.3 LC_SUMMARY COLUMN MAP
The column positions assume a 1‐space delimiter between columns. The mapping is the same as
that used for the public releases of the LCDB found on the MinorPlanet.info web site.

See the notes after the lc_details column map regarding the flags used to indicate the
source/method used for certain values

Field        Format      Pos     Notes
Number         I7          1‐7   MPC‐assigned number; empty if no number assigned
EntryFlag      A1            9   “*” indicates a vetted new/revised record since the last
                                 major release. Other flags are used during the vetting
                                 process that will be removed for the annual vetted release.
Name           A30       11‐40   MPC‐assigned name, or designation if not named
Desig          A20       42‐61   MPC primary designation, if assigned
Family         A8        63‐70   The orbital group or collisional family
CSource        A1           72   Flag indicating source for taxonomic classification
Class          A10       74‐83   The taxonomic class
DiamSource     A1           85   Flag indicating the source for the diameter
DiamFlag       A1           87   Flag (e.g., < or >) that qualifies the diameter
Diam           F8.3      89‐96   Adopted Diameter (km)
HSource        A1           98   Flag indicating the source of the H value
H              F6.3    100‐105   Adopted absolute magnitude H
HBand          A2      107‐108   The photometric band of H
GSource        A1          110   Flag indicating the source of the G value
G              F6.3    112‐117   Adopted phase slope parameter (G or G1; see 3.1.1)
G2             F6.3    119‐124   Adopted phase slope parameter (G2; 3.1.1)
AlbSource      A1          126   Flag indicating the source of the albedo value
AlbFlag        A1          128   Flag (e.g., < or >) qualifying the albedo value
Albedo         F6.4    130‐135   Adopted Albedo (same band as H)
PFlag          A1          137   Period qualifier
Period         F13.8   139‐151   Rotation period, in hours; usually synodic
PDescrip       A15     153‐167   Description of period if PFlag = 'D'; e.g., "long"
AmpFlag        A1          169   Amplitude flag, e.g., > or <
AmpMin         F4.2    171‐174   Minimum reported amplitude
AmpMax         F4.2    176‐179   Maximum reported amplitude
U              A2      181‐182   Lightcurve Quality
Notes          A5      184‐188   Qualifying flags for record
Binary         A3     190‐192      ? = Suspected; B = Binary; M = Multiple; blank if none
Pole           A3     194‐196      Y/N Y = Pole position reported in spin axis table
Survey         A5     198‐202      Type of Survey if result from large survey programs, e.g.,
                                   PTF, WTF, Kepler, TESS, PanSTARRS, etc.
                                   See Notes below and Section 5
ExNotes        A3      204‐206 Y/N Y = Entry in lc_exnotes table
Private PRI A3         208‐210 Y = Unpublished, contact named observer
                      to request details
The Min/Max Amplitude values are based only on detail lines that have a U >= 2‐ ratings. If the
code is empty or U <= 1+, the detail line min/max amplitudes are not considered.


4.1.4 LC_DETAILS COLUMN MAP
The column positions assume a 1‐space delimiter between columns. The mapping is different
from that used for the public releases of the LCDB found on the MinorPlanet.info web site
because this report contains the error values.

Field    Format         Pos        Notes
Number     I7             1‐7      MPC‐assigned number; empty if no number assigned
Name       A30           9‐38      MPC‐assigned name, or designation if not named
SumPer     F13.8        61‐73      Period from summary table, hours
SumAmp     F4.2         75‐79      AmpMax from summary table
ShortRef   A30         81‐110      Short Reference from lc_references table
WorkedAs   A20        112‐131      The name or designation at the time of the observations
DateObs    A10        133‐142      Mid‐date (yyyy‐mm‐dd, 0h UT) of mid‐observations
PABL       F5.1       144‐148      Phase angle bisector longitude at DateObs
PABB       F5.1       150‐154      Phase angle bisector latitude at DateObs
Phase      F5.1       156‐160      Solar phase angle at DateObs (always positive)
Family     A5         162‐166      Orbital group/collisional family, if reported
Class      A5         168‐172      Taxonomic class, if reported
DiamMeth   A1             174      Method used to find reported diameter, if any
Diam       F8.3       176‐183      Diameter (km), if reported
DiamErr    F8.3       185‐192      Error in Diam (km), if reported
HMeth      A1             194      Method used to find H, if reported
H          F6.3       196‐201      Absolute magnitude (H), if reported
Herr       F6.3       203‐208      Error in H, if reported
HBand      A2         210‐211      Color band for H (see notes)
GSource    A1             213      Method used to find reported G, if any
G             F6.3    215‐220      Phase slope parameter (G or G12), if reported
GErr          F6.3    222‐227      Error in G, if reported
G2            F6.3    229‐234      Phase slope parameter (G or G12), if reported
G2Err         F6.3    236‐241      Error in G, if reported
AlbSource     A1          243      Method to find albedo, if reported
AlbFlag       A1          245      Flag qualifying albedo value, e.g., < or >
Albedo        F6.4    247‐252      Albedo, if reported (same band as H, if reported)
AlbErr        F6.4    254‐259      Error in albedo, if reported
PFlag         A1          261      Period qualifier
Period        F13.8   263‐275      Rotation period, hours; usually synodic
PerErr        F10.8   277‐286      Error in period, hours
PDescrip      A15     288‐302      Description of period if PFlag = 'D'; e.g., "long"
AmpFlag       A1          304      Amplitude qualifier, e.g., > or <
AmpMin        F4.2    306‐309      Minimum amplitude, if reported
AmpMax        F4.2    311‐314      Maximum amplitude, if reported
AmpErr        F4.2    316‐319      Error in AmpMax, if reported
U             A2      321‐322      Lightcurve Quality
Notes         A5      324‐328      Qualifying flags for record
IsBinary      A1          330      ? = Suspected; B = Binary; M = Multiple; blank if none
Pole          A1          332      Y/N Y = Pole position reported in spin axis table
Survey        A5      334‐338      Type of Survey if result from large survey programs, e.g.,
                                   PTF, WTF, Kepler, TESS, PanSTARRS, etc.
                                   See Notes below and Section 5
ExNotes       A1           340     Y/N Y = Entry in lc_exnotes table
Private       A1           342     Y/N Y = Private record. The results have been supplied
                                   for the LCDB authors' use only; the only entry will be
                                   the number/name/desig and U code, if there is one.

NOTES
The values for H, G, Diameter, and albedo may have been measured, calculated (e.g., Diameter
from H and albedo), or assumed. For IR surveys, e.g., WISE, the H value was often assumed based
on the value from the MPCORB table or some other source. Another possible case is a value for
H determined using an assumed value for G. See Warner et al. (2009) for a more detailed
explanation of the source/method flags.

Table 4.1.4.1 Taxonomic Class Source/Method Flags
A      Assumed based on orbital group
L      Taken from a details table entry
S      SMASS (Bus and Binzel, 2002a; 2002b)
T      Tholen (1984)
Table 4.1.4.2 H Method/Source Flags
A      From Lowell ASTORB table
D      Derived from diameter and albedo
E      Estimated
L      Taken from a details table entry
M      From MPCORB table
S      From SIMPS (Tedesco et al., 2004)
T      Transformed (Usually a details entry converted from H_R to H_V)
W      From WISE (Mainzer et al., 2019)

Table 4.1.4.3 H Color Band
This indicates the color band in which the value for H was found
Blank      Johnson V
B          Johnson B
V          Johnson V
R          Cousins R
I          Cousins I
SU         Sloan u´
SG         Sloan g´
SR         Sloan r´
SI         Sloan i´
SZ         Sloan z´

Table 4.1.4.4 G Method/Source Flags
A      From Lowell ASTORB table
D      Default for taxonomic class (see Warner et al., 2009)
G      Based on H‐G12 (no value for G2) or H‐G1,G2 (value for G2) system
L      From an entry in the details table
M      From the MPCORB table
P      From Pan‐STARRS (Veres et al., 2015)
W      From WISE (Mainzer et al., 2019)

Table 4.1.4.5 Diameter Qualifier Flags
<      Diameter is a maximum
>      Diameter is a minimum

Table 4.1.4.6 Diameter Method/Source Flags
C      Calculated from albedo and H
D      Derived from albedo and H (after using Harris and Harris, 1997)
K      From AKARI (Usui et al., 2011)
L      Taken from a details table entry
S      From SIMPS (Tedesco et al., 2004)
T      Thermal (determined from IR observations)
W      From WISE (Mainzer et al., 2019)
Table 4.1.4.7 Albedo Method/Source Flags
A      Assumed (based on orbital group or taxonomic type)
D      Derived from H and diameter
K      From AKARI (Usui et al., 2011)
L      From a details table entry
S      From SIMPS (Tedesco et al., 2004)
W      From WISE (Mainzer et al., 2019)

Table 4.1.4.8 Period Flags
<      Period is a maximum value
>      Period is a minimum value
D      Indeterminate period described in the PDescrip field
S      Period is sidereal (default is synodic)


4.1.5 FIELD (FLAG) CODES USED IN SUMMARY AND DETAIL LINES
The flags appear in the data field immediately before the value they qualify. In most cases, they
are a single character.

Table 4.1.5.1 AMPFLAG (Amplitude Flag)
Blank    NONE
<        Less than
>        Greater than

Table 4.1.5.2 PFLAG (Period Flag)
Blank    NONE
<        Less than
>        Greater than
D        No numerical value, see P DESC field description
S        Sidereal period, default is no flag and synodic period
U        Uncertain, not the same as ambiguous where one or additional periods are reported.
         For example, the data did not allow finding a definite period and so the author(s)
         reported a "best guess."

Table 4.1.5.3 NOTES (single letter flag(s))
Blank    NONE
?        Usually tied with 'T' or 'A' flags to indicate uncertainty
–        Tied with T flag. See notes below.
<X>      Number max/min pairs per rotation, e.g., 3 is a trimodal lightcurve.
A        Ambiguous period (see lc_ambiguous table for details)
C        Occultation observation
D        Period determined by us that differs from that given in the original publication
E        Occultation observation (usually when reporting a diameter)
H        Space telescope observations (optical)
I        IR/Thermal observations (e.g., Spitzer)
M        Polarimetric observation
N        No lightcurve published
O        Adaptive optics observation
P        Photographic photometry
R        Radar observation
S        Spectroscopic
T        Tumbling (NPA rotation ‐ see lc_npa table for details and notes below).
U        Undetermined period/amplitude. Used to indicate that data were obtained but no
         period or amplitude was given, usually because there was no obvious trend. A raw plot
         of the data may have been included.
V        Visual photometry
Z        Multiple data sources

The 'A' and 'T' notes flags are used to call the reader's attention to the lc_ambiguous or lc_npa
tables, respectively. They should not be taken as stand‐alone information. Instead, consider them
footnote numbers in the body of a main text. The other reports (and original references) are the
actual footnotes.

The A flag does not appear in the summary line unless the summary line value itself represents
an ambiguous solution, i.e., just because a details line may report an ambiguous period does not
mean that the summary period is also ambiguous.

The T flag currently has four possible qualifiers

Blank The asteroid has a PAR < ‐1, i.e., it is definitely tumbling.
       Example: T

?      Possible tumbler. There is some evidence that the asteroid might be a tumbler. It may
       carry a PAR = 0 to ‐1. See the discussion for the lc_npa table for the meaning of the PAR
       codes.
       Example: T?

0      The tumbling damping time scale (see Pravec et al, 2005) is long enough that tumbling
       might be expected, but observations are not sufficient to substantiate either tumbling or
       not tumbling, PAR = 0.
       Example: T0

–      The tumbling damping time scale is long enough that tumbling might be expected, but
       observations indicate that the object is NOT tumbling, i.e., PAR >= 1.
       Example: T‐
+      The tumbling damping time scale is short enough that tumbling would not seem likely,
       however observations indicate that it may be tumbling or actually is tumbling, i.e.,
       PAR = < 0.
       Example: T+

We include the expanded tumbling notes to call attention to what we consider to be an important
aspect in the study of YORP spin up/down theories. This is done by making known any asteroids
that are or are strongly believed to be tumbling as well as those that should be and aren't or are
and shouldn't be.

The W flag is included so that those doing statistical studies can include or exclude the results
from these surveys. Such surveys can introduce significant biases by "cherry picking" the best
results from a large pool and so skew overall rotational statistics. See the paper by Warner and
Harris (2011, Icarus).


4.1.6 DATA SUITABLE FOR ROTATION RATE STUDIES
As noted in Warner et al. (2009), only those objects with a U code of 2- or greater in the
lc_summary table, i.e., U = 2‐, 2, 2+, 3‐, or 3, should be used for rotational rate studies and, unless
there is a specific reason otherwise, the summary line period should be used instead of one of
the periods in the details table.


4.2.0 LC_AMBIGUOUS (AMBIGUOUS PERIODS)
This table includes any record where the notes flag in a summary and/or detail record indicates
an ambiguous period.

There is not always a direct cross‐connection between the summary and details entries. For
example, it's possible to have a summary line without the ambiguous period flag but one or more
of the Details lines to have the flag. In this case, we judge that the ambiguity has been resolved
by subsequent observations, but retain the ambiguous flag in the detail line for historical
accuracy.

In turn, if the summary line is flagged as ambiguous, this does not mean that any of the details
lines are also flagged as such. In that case, it means that no one solution sufficiently stands out
and so the one that is reported on the summary line is considered to be only the most probable
solution.

The first line for a given object is the Summary line, which contains the number and name of the
object and the adopted period and amplitude. As noted above, the details lines(s) may not agree
with the summary line.
4.2.1 LC_AMBIGUOUS COLUMN MAPPING
Field      Format        Pos        Notes
Number       I7             1‐7     MPC‐assigned number; empty if no number assigned
Name         A30           9‐38     MPC‐assigned name, or designation if not named
SumPer       F13.8        61‐73     Period from summary table, hours
SumAmp       F4.2         75‐79     AmpMax from summary table
SumNotes     A5           81‐85     Qualifying flags for the summary record
DetNotes     A5           87‐91     Qualifying flags for the details record
ShortRef     A30         93‐122     Short Reference from lc_references table
DateObs      A10        124‐133     Mid‐date (yyyy‐mm‐dd, 0h UT) of observations
Period1      F13.8      135‐147     Preferred period, hours, from details record
Period1Err   F13.8      149‐161     Error in preferred period, if reported
Amp1         F4.2       163‐166     Preferred amplitude from details record
Amp1Err      F4.2       168‐171     Error in amplitude, if reported
Period2      F13.8      173‐185     First alternate period, hours
Period2Err   F13.8      187‐199     Error in period, if reported
Amp2         F4.2       201‐204     First alternate amplitude
Amp2Err      F4.2       206‐209     Error in amplitude, if reported
Period3      F13.8      211‐223     Second alternate period, hours
Period3Err   F13.8      225‐237     Error in period, if reported
Amp3         F4.2       239‐242     Second alternate amplitude
Amp3Err      F4.2       244‐247     Error in amplitude, if reported
Period4      F13.8      249‐261     Third alternate period, hours
Period4Err   F13.8      263‐275     Error in period, if reported
Amp4         F4.2       277‐280     Third alternate amplitude
Amp4Err      F4.2       282‐285     Error in amplitude, if reported


4.3.0 LC_BINARY (BINARY/MULTIPLE ASTEROIDS)
This table includes those asteroids that are known or suspected binaries. This is not meant to be
a comprehensive compilation of data for binary asteroids. Visit the URL given in section 2.2.0 for
a page that provides more details as well as links to the original journal articles.
Each line indicates the type of binary. There are four broad categories:

A      Fully‐asynchronous
       Example: 1509 Esclangona The satellite's rotation period is different from its orbital
       period. In this case, the orbital period is given along with the independent rotation period
       and lightcurve amplitude of the satellite, if available.

S      Singly‐asynchronous:
       Example: 5905 Johnson. The satellite's rotation period and orbital period are the same,
       i.e., they are tidally‐locked, but different from the primary's spin period. In this case, nly
       an orbital period is given. The lightcurve may be flat or bowed between events. If flat, the
       presumption is that the satellite is nearly spheroidal and the rotation is still tidally‐locked
       to the orbit. If the lightcurve shows an overall "bowed" shaped, this is presumed to
       indicate a significantly elongated satellite.

F      Fully‐synchronous
       Example: Pluto/Charon, 90 Antiope. The rotation period of the primary and satellite are
       the same and is the same as the orbital period of the satellite. In this case, the primary
       rotation period and lightcurve amplitude is given and matches the orbital period of the
       two bodies. No secondary period is given.

U      Undetermined
       This is usually reserved for binaries discovered by imaging with Hubble or very large
       ground‐based telescopes. In most cases, the orbital parameters are not or very poorly
       known and there are no lightcurves to determine the actual type of binary, e.g., if the
       satellite is tidally locked to its orbital period.

In some asynchronous systems, it is not always possible to determine with certainty which of the
two periods is due to the primary and which is due to secondary. In these cases, we are forced to
give the period and amplitude of one body as that of the "primary" and the other period and
amplitude as that of the "secondary" when, in fact, the roles may be reversed from our selection.

For multiple systems and in _most_ cases, the satellite information is for the first one discovered.
In some cases, e.g., 3749 Balam, the first discovery was for a satellite with a long orbital period
of 1920 hours. It is assumed that the satellite's rotation is not equal to the orbital period. A
second satellite was found that has a rotation period that is tidally‐locked to its orbital period of
about 33.4 hours.

Each line also gives the primary rotation period and amplitude and secondary/orbital
periods/amplitudes as appropriate. If available, the estimated effective diameter ratio (Ds/Dp) is
given, as are the ratio of the semi‐major axis of the satellite orbit to the diameter of the primary
(A/Dp).
The Ds/Dp ratio is a minimum in most cases since total eclipses were not seen in the satellite's
lightcurve. The DsDpFlag qualifies this value, e.g., < or >. If there is no flag and there is a Ds/Dp
value, assume '='.


4.3.1 SECONDARY VS. ORBITAL PERIOD
In some cases, there is only a secondary period ("SecPer") given; in others, only an orbital period
("OrbPer"); and in others, both periods are given. The case when "PerOrb" is given is usually the
result of timing of mutual events (occultations and/or eclipses) and so there will be at least an
"EventMax" value given.

When only "SecPer" is reported, then the lightcurve was defined by two periods, with the second
period attributed to the rotation of a satellite but the viewing geometry did not allow mutual
events. The second period might also be due to low‐level tumbling. Regardless, without mutual
events or other definitive confirmation, the asteroid will likely be classified as "suspected" and
not confirmed.

When both periods are reported, then the secondary period is likely due to the presence of a
third body in the system. Unless separate mutual events or other definitive evidence is provided,
they system will be classified as "binary" and not "multiple."


4.3.2 LC_BINARY COLUMN MAPPING
Field        Format        Pos        Notes
Number         I7            1‐7      MPC‐assigned number; empty if no number assigned
Name           A30          9‐38      MPC‐assigned name, or designation if not named
SumBin         A1             40      ? = suspected; B = binary; M = multiple
SumPer         F13.8       42‐54      Period from summary table, hours
SumAmp         F4.2        56‐59      AmpMax from summary table
ShortRef       A30         61‐90      Short reference from lc_references table
DateObs        A10        92‐101      Mid‐date (yyyy‐mm‐dd, 0h UT) of observations
DetBin         A1            103      ? = suspected; B = binary; M = multiple
BinType        A1            105      A = fully‐asynchronous; S = singly‐asynchronous;
                                      F = fully‐synchronous
PrimPer        F13.8     107‐119      Rotation period of the primary, hours
PrimPerErr     F13.8     121‐133      Error in period, if reported
PrimAmp        F4.2      135‐138      Maximum amplitude of the primary lightcurve
PrimAmpErr     F4.2      140‐143      Error in primary amplitude, if reported
SecPer         F13.8     145‐157      Secondary period (see section above)
SecPerErr      F13.8     159‐171      Error in secondary period, if reported
SecAmp       F4.2     173‐176      Amplitude of secondary period lightcurve
SecAmpErr    F4.2     178‐181      Error in amplitude, if reported
OrbPer       F13.8    183‐195      Orbital period of the first satellite, hours
OrbPerErr    F13.8    197‐209      Error in period, if reported
EventMin     F4.2     211‐214      Shallowest amplitude of the mutual events
EventMax     F4.2     216‐219      Deepest amplitude of the mutual events
DsDpFlag     A1           221      Qualifier for Ds/Dp, e.g., < or >
DsDp         F4.2     223‐226      Ratio of first satellite/primary effective diameters
DsDpErr      F4.2     228‐231      Error in Ds/Dp value
ADp          F5.2     233‐237      Ratio of first satellite orbital semi‐major axis to primary
                                   diameter
ADpErr       F5.2     239‐243      Error in Ds/Dp ratio


4.4.0 LC_COLORINDEX
Unless the lc_exnotes table indicates otherwise, the bands are on the Johnson‐Cousins BVRI,
Sloan griz, or ATLAS oc systems.


4.4.1 LC_COLORINDEX COLUMN MAPPING
Field     Format      Pos       Notes
Number      I7           1‐7     MPC‐assigned number; empty if no number assigned
Name        A30         9‐38     MPC‐assigned name, or designation if not named
SumPer      F13.8      40‐52     Rotation period from summary table, hours
SumAmp      F4.2       54‐57     AmpMax from summary table
ShortRef    A30        59‐88     Short reference from the lc_references table
DateObs     A10        90‐99     Mid‐date (yyyy‐mm‐dd, 0h UT) of the observations
DetPer      F13.8    101‐113     Rotation period from details record, if reported; hours
DetPerErr   F13.8    115‐127     Error in rotation period, hours
DetAmp      F4.2     129‐132     Lightcurve amplitude, if reported
DetAmpErr F4.2       134‐137     Error in amplitude
BV          F6.3     139‐144     B‐V color index
BVErr       F6.3     146‐151     B‐V error
BR          F6.3     153‐158     B‐R color index
BRErr       F6.3     160‐165     B‐R error
VR          F6.3     167‐172     V‐R color index
VRErr       F6.3     174‐179     V‐R error
VI          F6.3     181‐186     V‐I color index
VIErr        F6.3      188‐193     V‐I error
SGR          F6.3      195‐200     g‐r color index
SGRErr       F6.3      202‐207     g‐r error
SRI          F6.3      209‐214     r‐i color index
SRIErr       F6.3      216‐221     r‐i error
SIZ          F6.3      223‐228     i‐z color index
SIZErr       F6.3      230‐235     i‐z error
ATLco        F6.3      237‐242     ATLAS c‐o color index
ATLcoErr     F6.3      244‐249     ATLAS c‐o error


4.5.0 LC_DIAMETERS
This table is provided for those wanting to quickly dissect and compare diameters reported in the
summary and details tables. It includes "orphaned" summary records (see Section 3.2.0/6.0.1,
"ORPHAN RECORDS").


4.5.1 LC_DIAMETERS COLUMN MAPPING
Field    Format          Pos       Notes
Number     I7              1‐7     MPC‐assigned number; empty if no number assigned
Name       A30            9‐38     MPC‐assigned name, or designation if not named
SumPer     F13.8         40‐52     Rotation period from summary table, hours
SHBand     A2            54‐55     Photometric band for H from the summary record
SumH       F6.3          57‐62     Absolute magnitude (H) from summary record
SumG       F6.3          64‐69     Phase slope parameter (G) from summary record
SumPv      F6.4          71‐76     Albedo from summary record
SumDiam    F8.3          78‐85     Diameter (km) from the summary record
SumNotes   A5            87‐91     Qualifying flags for the summary record (See Notes section
                                   after lc_details)
ShortRef      A30       93‐122     Short reference from lc_references table
DateObs       A10      124‐133     Mid‐date (yyyy‐mm‐dd, 0h UT) of observations
DetPer        F13.8    135‐147     Rotation period from details record, hours
DetPerErr     F13.8    149‐161     Error in rotation period
DetHSource    A1           163     Flag indicate method used to find reported diameter
DHBand        A2       165‐166     Photometric band of H
DetH          F6.3     168‐173     Absolute magnitude (H) from details record
DetHErr       F6.3     175‐180     Error in H
DetGSource    A1           182     Flag indicating method use to find reported G
DetG           F6.3     184‐189     Phase slope parameter (G or G12) from details record
GetGErr        F6.3     191‐196     Error in G
DetPv          F6.4     198‐203     Albedo from details record, same band as H
DetPvErr       F6.4     205‐210     Error in albedo
DetDiam        F8.3     212‐219     Diameter (km) from details record
DetDiamErr     F8.3     221‐228     Error in diameter
DetNotes       A5       230‐234     Qualifying flags for the details record (See Notes section
                                    after lc_details)


4.6.0 LC_FAMILYLOOKKUP
The LCDB recognizes more than 200 dynamical families and orbital groups. The list of dynamical
families is taken from Nesvorny (2015) and Nesvorny et al. (2015) and the AstDys (2021) web site.
Family assignment for individual objects is based on the list of family membership from the
AstDys site. When the object is not in the family membership list, it is assigned to a broad orbital
group. See “3.1.2      FAMILY/GROUP MEMBERSHIP, DEFAULT ALBEDOS, AND TAXONOMIC
CLASS.”


4.6.1 LC_FAMILYLOOKUP COLUMN MAPPING
Field        Format       Pos       Notes
Number         A7           1‐7     Family number. Values < 1000 are from Nesvorny (2015).
Parent         I7          9‐15     MPC‐assigned number of the parent (largest) body of the
                                    of the family. 0 if an orbital group and not a family.
Family Name A20           17‐36     Name of the parent body of the family or the broad orbital
                                    group.
Count          I7         38‐44     Number of members in a family (from AstDys site list).
Albedo         F6.3       46‐51     The default albedo for the family/group.
Err            F6.3       53‐58     The estimated error in the albedo.
G              F6.2       60‐65     The default value for G (on H‐G system) for the family
                                    or group.
Err            F6.2       67‐72     The estimated error for G.
Class          A10        74‐83     The default taxonomic class.

If Class is appended with an asterisk, e.g., C*,the value is based on orbital location and not that
of the parent body.
4.7.0 LC_EXNOTES
The lc_exnotes table contains extended "free‐form" notes for summary and/or details records.
These entries allow expanded information that cannot be given by a simple, single‐character flag.

In some cases, only the summary record has an extended note for an object. In this case, the
output line will include the summary information given in the column mapping but the rest of
the fields will have the default <no data> entry.

If there is no summary extended note for a given asteroid but one or more details records with
notes, then ‐ for each detail record ‐ the summary number and name are included, the summary
notes field uses the default <no data>, and the data for the given detail record are given.

If there are both summary and details extended notes, then the first line includes only the
summary extended note and uses the default <no data> for the rest of the line. Subsequent lines
for the asteroid do not include the summary note but do include the detail record note.

In short, no line will contain BOTH a summary and detail extended note.

The summary and details table ExNotes fields are defined as varchar(1024) in their MySQL tables.
In practice, however, the longest entries is < 128 characters. Even so, keep in mind that a full‐
length line could exceed 1100 characters.


4.7.1 LC_EXNOTES COLUMN MAPPING
The column mapping below allows for the maximum length of each field. In practice, a delimited
(e.g., comma or semi‐colon) file with one record per line will be much shorter than the maximum
length.

The maps below do not account for the <no data> flags, usually '‐' for a string value and ‐99 for
the Number field if the asteroid is not numbered. For a summary note only record, the fields after
"SumExNotes" would use the <no data> value. For details note only record, the "Number" and
"Name" fields would have values but the "SumExNotes" field would use the <no data> value.
SUMMARY EXTENDED NOTE

Field  Format Pos Notes
Number      I7       1‐7            MPC‐assigned number; empty if no number assigned
Name        A30     9‐38            MPC‐assigned name, or designation if not named
SumExNotes A1024 40‐1063            Extended note for summary record

WITH DETAIL EXTENDED NOTE

Field         Format        Pos       Notes
Number          I7            1‐7     MPC‐assigned number; empty if no number assigned
Name            A30          9‐38     MPC‐assigned name, or designation if not named
WorkedAs        A20         40‐59     The name or designation used by the original authors.
                                      This may or may not be the same as the current
                                      MPC‐assigned name and/or designation
ShortRef        A30        61‐90      The short reference in the publications table
DetExNotes      A1024    92‐1115      Extended note for detail record


4.8.0 LC_NPA (NON‐PRINCIPAL AXIS ROTATION ‐ TUMBLING)
This table not just confirmed tumbling asteroids but those that are suspected, those that "should
be" tumbling but apparently are not, and those that are tumbling that "should not be" tumbling.

In the table, the first period (DetPeriod) is usually the dominant one. Whether or not it is the
period of rotation or precession cannot often be established.


4.8.1 PAR RATING
The PAR rating is adopted from Pravec et al. (2005). See also Pravec et al. (2010), in which a
revised set of damping times is developed. These so‐called "short" damping times are several
times shorter than in the original paper and are preferred.

Here is a brief description of the PAR codes. Pravec et al. for a more detailed explanation.

‐4         Physical model of the NPA rotation constructed
‐3         NPA rotation reliably detected with the two periods resolved. There may be some
           ambiguities in one or both periods.
‐2         NPA rotation detected based on deviations from a single period but the second period
           is not resolved.
‐1         NPA rotation possible, i.e., deviations from a single period are seen, but not
           conclusively.
 0         Insufficient data to determine if rotation is PA or NPA
+1         PA rotation is consistent with the data but coverage is insufficient.
+2         PA rotation likely, or deviations from PA are small with some overlapping data fitting
           a PA rotation period.
+3         PA rotation quite likely
+4         PA spin vector obtained.
Entries with a positive number are rare and used when the asteroid was thought to be tumbling
but further examination showed it was likely in PA rotation, or when the damping time to PA
rotation is sufficiently long that the given asteroid would more likely be in NPA than PA rotation.


4.8.2 LC_NPA COLUMN MAPPING
Field        Format    Pos      Notes
Number         I7           1‐7   MPC‐assigned number; empty if no number assigned
Name           A30         9‐38 MPC‐assigned name, or designation if not named
SumPer         F13.8      40‐52 Rotation period from summary record, hours
SumAmp        F4.2        54‐57   Maximum lightcurve amplitude from the summary record
SumNotes       A5         59‐63   Qualifying flags for the summary record (See Notes section
                                  after lc_details)
ShortRef       A30        65‐94 Short reference from the lc_references table
DateObs        A10       96‐105 Mid‐date (yyyy‐mm‐dd, 0h UT) of observations
DetPeriod      F13.8    107‐119 First (dominant) period from the details record, hours
DetPerErr      F13.8    121‐133 Error in period
DetAmp         F4.2     135‐138 Amplitude associated with first period
DetAmpErr      F4.2     140‐143 Error in amplitude
DetPer2        F13.8    145‐157 Second period, either precession or rotation, hours
DetPer2Err     F13.8    159‐171 Error in second period
DetAmp2        F4.2     173‐176 Amplitude associated with second period, rarely reported
DetAmp2Err     F4.2     178‐181 Error in amplitude
PAR            A2       183‐184 PAR rating under Pravec et al. system
DetNotes       A5       186‐190 Qualifying flags for the details record (See Notes section
                                  after lc_details)
4.9.0 LC_REFERENCES (REFERENCES PUBLICATIONS)
The LC_REFERENCES table contains the Bibcode, short reference, and literature citation for each
reference in the other data tables. The literature citation limits the number of authors to five. If
there are more than five, the fifth "author" is "et al." In recent years, some journals stopped using
page numbers but article ids. These are shown as "Axx" with xx being the article id. Where page
numbers are given, both starting and ending numbers are given.

As is customary, if there is only one publication for a given author in a given year, the short
reference does not include a letter after the year, e.g., Warner (2018). If there is more than one
publication, then the entries are appended with 'a' through 'z'. So far, it has not been necessary
to devise a method for someone having more than 26 publications in a single year.

Some short references are appended with 'web', e.g., Warner (2018web). This indicates the
results were posted on a web site, hopefully in anticipation of publication in a permanent journal.
This also prevents a conflict should an author publish at least 23 papers ('w') ‐and‐ also posted
pending results on a web site.

Current Bibcodes are 19 characters long. The field allows one more character should expansion
be required in the future. For the time being, the current table does ‐not‐ pad the Bibcode to 20
characters.

The Citation field allows up to 255 characters. However, since the LCDB limits the list to five
authors, the likelihood of this field exceeding 80 characters is very small.


4.10.1 LC_REF COLUMN MAPPING
Field        Format Pos          Notes
BibCode      A20     1‐19        19‐character BibCode
ShortRef     A30    21‐50        Primary author and year, e.g., Warner (2018a)
Citation     A255 52‐306         Literature citation

N.B. Initials for names are packed, e.g., Warner, B.D. and not Warner, B. D.


4.10.0 LC_SPINAXIS (POLE SOLUTIONS)
This table includes any asteroid for which spin axis information has been reported. See section
2.1.1, "Spin Axis Catalogs" for additional resources and references.

The table lists up to four pole solutions. This allows for the known issues with lightcurve inversion,
especially when the object has a low orbital inclination. Generally, it's not uncommon to have
two solutions that differ by 180° in longitude but have nearly the same latitude. However, there
are other cases where the latitudes are mirrored about the equator and the longitudes are
similar. There is also the possibility that both longitude and latitude are mirrored, thus having
four possible solutions.


4.10.1 Q (QUALITY) RATING
The Q value gives our assessment of the quality of the pole solution. It is adopted from the rating
system used in Kryszczynska et al. (2007, Icarus 192, 223‐237).

0      Either wrong or very uncertain determination
1      Possible but not certain pole determination. This will most often appear when a limited
       number of data sets is used, especially if methods other than lightcurve inversion are
       involved.
2      Good determination, based on large dataset. The solution consists of one or two solutions
       (and possibly their 180° mirrors). If two solutions, they may differ in both longitude and
       latitude but not by the simple 180° mirror.
3      Very good determination, based on large dataset, an ambiguity of about 180° in pole
       longitude might appear.
4      Excellent determination, pole position confirmed by methods based on independent
       datasets (for example, lightcurves and radar data, lightcurves and spacecraft fly‐by).
P      A prograde rotation has been determined but no specific pole position has been
       determined. This will be followed by a 0 or 1, indicating the quality of the determination.
R      A retrograde rotation has been determined by no specific pole position has been
       determined. This will be followed by a 0 or 1, indicating the quality of the determination.

If the Q value is blank, the given pole solution has not yet been reviewed under the new rating
system.


4.10.2 SPECIAL ENTRIES
Sometimes an entry will have a value of L1 = ‐1. This indicates no longitude was reported. The
value of B1 has two interpretations

If the latitude is ‐99.9, then no latitude was reported. This entry must have a Q value of P or R,
meaning prograde or retrograde rotation was determined. This is usually by seeing how the
synodic rotation period changed before, at, and after opposition. Other techniques than
lightcurve inversion can also produce a sense of rotation but no longitude/latitude pair.

If the latitude is || <= 90.0, then a latitude only solution was found, although it is usually more
a "best guess" and can have substantial errors. Again, the Q rating must be P or R, which is
assigned on the basis that positive latitudes imply prograde rotation and negative latitudes imply
retrograde rotation.
A negative longitude will not appear for Long2‐Long4.


4.10.3 LC_SPINAXIS COLUMN MAPPING
Field         Format    Pos           Notes
Number       I7          1‐7      MPC‐assigned number; empty if no number assigned
Name         A30        9‐38      MPC‐assigned name, or designation if not named
SumPer       F13.8     40‐52      Rotation period from summary record, hours
SumAmp       F4.2      54‐57      Maximum lightcurve amplitude from the summary record
ShortRef     A30       59‐88      Short reference from lc_references table
DateObs      A10       90‐99      Mid‐date (yyyy‐mm‐dd, 0h UT) of observations
Long1        F5.1    101‐105      Ecliptic longitude of the preferred pole
Long1Err     F5.1    107‐111      Error in Long1
Lat1         F5.1    113‐117      Ecliptic latitude of the preferred pole (always includes + or ‐)
Lat1Err      F5.1    119‐123      Error in Lat1
Long2        F5.1    101‐105      Ecliptic longitude of first alternate pole
Long2Err     F5.1    107‐111      Error in Long2
Lat2         F5.1    113‐117      Ecliptic latitude of first alternate pole (always includes ±)
Lat2Err      F5.1    119‐123      Error in Lat2
Long3        F5.1    101‐105      Ecliptic longitude of second alternate pole
Long3Err     F5.1    107‐111      Error in Long3
Lat3         F5.1    113‐117      Ecliptic latitude of second alternate pole (always includes ±)
Lat3Err      F5.1    119‐123      Error in Lat3
Long4        F5.1    101‐105      Ecliptic longitude of third alternate pole
Long4Err     F5.1    107‐111      Error in Long4
Lat4         F5.1    113‐117      Ecliptic latitude of third alternate pole (always includes ±)
Lat4Err      F5.1    119‐123      Error in Lat4
SidPer       F13.8   125‐137      Sidereal period of spin axis solution (for long/lat1)
Model        A1          139      Y/N; Y = Shape model reported.
Q            A4      141‐144      Quality of pole solution (see section 4.10.1)
5.0.0 HANDLING SURVEY DATA

5.0.1 INTRODUCTION
Up until the 2020 February release, the LCDB contained two fields related to large surveys:
WideField and SparseData. Due to the coming of surveys that far exceed the data/results density
of earlier works, and the desire to qualify the results from lightcurve inversion programs using
dense and/or sparse data, the two fields were merged into a single field: "Survey", which is up to
five alphanumeric characters long.

The previous discussions on wide‐field and sparse data sets below have been modified to include
the revised and new definitions. To understand the foundation for the differentiation between
dense lightcurves and among the various surveys, you are referred to two papers:

   Warner, B.D., Harris, A.W. (2011) "Using sparse photometric data sets for asteroid lightcurve
   studies." Icarus 216, 610‐624.

   Harris, A.W., Pravec, P., Warner, B.D. (2012) "Looking a gift horse in the mouth: Evaluation of
   wide‐field asteroid photometric surveys." Icarus 221, 226‐235.

In essence, the reason for categorization is because of the large biases introduced into rotational
study statistics by the surveys because they do not analyze and report results on _every_ object
observed, but small, select data sets, those where there was at least some success in arriving at
"reliable" results.

For example, in Waszczak et al. (2015), they reported observations on more than 50,000
asteroids but found less than 10,000 "reliable" periods, or about a 16% success rate. Chang et al.
(2015) had about a 27% success rate. As a result, statistical studies should use most survey data
with some caution, if at all, since those data sets have the potential for adding substantial biases,
e.g., those against super‐fast or super‐slow rotators, tumblers, binary objects, and, probably
most significantly, against objects with low amplitudes, i.e., A < ~0.10‐0.12 mag.

Because of the large amounts of lightcurve results reaching the literature, the LCDB authors
adopted the policy of assigning U = 2 to any lightcurve from most survey data sets where the
authors claimed a "reliable" period and then flagging the entry so that end‐users can decided
whether or not to include the results in their research.

When the authors of a paper reported periods that were not considered reliable, those results
were assigned U = 1 in the LCDB. Where no period was reported, what information that was
available, e.g., amplitude, was entered and no U code was assigned.

Some surveys have provided the phased lightcurve plots, e.g., Waszczak et al., Erasmus, and Pal
et al. Over time, we hope to review those plots and assign more accurate assessments. Even so,
the results will still be flagged such that they are not included in the BASIC DATA set.
As with almost any attempt to put objects into narrow categories, there are gaps and overlaps.
Even so, the authors offer the following categories and samples of what data falls into each one.


5.1.0 WIDE‐FIELD SURVEYS

5.1.1 SPARSE WIDE‐FIELD
Summary/Details Listing Code: SWF
Included in BASIC DATA: NO

Definition: Data from a survey that gets a relatively small number of data points over a limited
            number of nights without regard to extending observations as needed to remove
            ambiguities or obtain a more accurate view of the period and/or nature (binary,
            tumbling) of the asteroid.

Examples: Palomar/Zwicky Transient Factory, TALCS, some K2


5.1.2 DENSE WIDE‐FIELD
Summary/Details Listing Code: DWF
Included in BASIC DATA: NO

Definition: Data from a survey that gets a large number of data points over an extended period
            of time. Usually this is possible only for surveys using space‐based telescopes. In
            these surveys, long‐period objects are often better covered and defined than by
            ground‐based surveys or programs. However, as with most other surveys, there is a
            significant possibility for analysis to be misled because of fixed observing cadences.
            For example, if a survey takes a image every 30 minutes and the rotation period of
            an object is almost exactly a half‐integral multiple of 30 minutes, then the sampling
            is capturing nearly the same point on the true lightcurve at each interval.

Examples: TESS, some K2


5.2.0 SPARSE DATA SETS

5.2.1 WIDE‐FIELD OR ALL‐SKY
Summary/Details Listing Code: SD
Included in BASIC DATA: NO

Definition: These are generally the result of surveys such as the Catalina Sky Survey, i.e., 2‐5
            data points a night on a few nights each lunation over several years. A more
            extensive example is the Lowell Lightcurve Database (Bowell et al., 2014) that
              includes hundreds of observations for some asteroid over 10‐15 years. The Gaia DR2
              data set is the most extensive one to‐date.

              It's usually only if using them in lightcurve inversion modeling that these data sets
              are used for period analysis and rarely, if at all, as stand‐alone sets.

Examples: Catalina Sky Survey, Lowell, Pan‐STARRS, Gaia DR2, ATLAS


5.2.2 NARROW/FIXED FIELD
Summary/Details Listing Code: SD
Included in BASIC DATA: NO

Definition: Data from a project that took repeated images of fixed intervals of a fixed field for
            a single night and, possibly, on a second night an adjacent field that accounted for
            average sky motion of the intended targets. The data sets can be used stand‐alone
            for period analysis but the results can be suspect, at best.

Examples: Dermawan et al. (2011)


5.3.0 LIGHTCURVE‐INVERSION
Lightcurve inversion (see Kaasalainen and Torppa (2001) and Kaasalainen et al. (2011) uses dense
and/or sparse data to try to find a model for an asteroid that will generate a lightcurve that
reproduces the actual data on more than one date.

Unless specific lightcurves are published (not just model/data comparisons), the NOTES field in
the record will include 'N' (lightcurve not published). If dense lightcurves are involved, it's
possible that one or more lightcurves were published under a different author and/or in the same
or other journal.

The reported period is almost always sidereal. In this case, the period flag (PFlag) field will get set
'S'. The U rating generally follows the guidelines set on the DAMIT site

    https://astro.troja.mff.cuni.cz/projects/damit/asteroids/

but the highest rating is still U = 3.


5.3.1 DENSE LIGHTCURVE INVERSION
Summary/Details Listing Code: LCI‐D
Included in BASIC DATA: YES
Definition: The modeling relied significantly, but not exclusively, on dense lightcurves (see
            definition in Warner and Harris, 2011) as well as sparse data.

             N.B. Results that are based solely on dense lightcurves are not included in this class
             and are not flagged in the Survey field.

Example: Hanus et al. (2016)


5.3.2 SPARSE LIGHTCURVE INVERSION
Summary/Details Listing Code: LCI‐W
Included in BASIC DATA: NO

Definition: The modeling relied exclusively on sparse data, such as Lowell, Gaia, Catalina, etc.

Example: Durech et al. (2016)
6.0.0 NUMBERS OF INTEREST
The numbers presented here are as of 2021 March 28.


6.0.1 ORPHAN RECORDS
No "orphan" records are in the lc_summary table. These are from publications that did not report
any observations towards finding a lightcurve period and/or amplitude. Some examples are most
of the IR survey papers (WISE, AKARI, SPITZER) that reported diameter and diameter. Others
include those reporting only color indices or taxonomic classification.

Likewise, if a summary record is orphaned, none of its detail lines are reported in the lc_details
table. However, the subtables, e.g., lc_binary and lc_colorindex, DO include the details lines for
orphaned summary records. In those tables, the number and name of the asteroid from the
summary line are included in each record.


6.1.0 SUMMARY TABLE ‐ OVERVIEW
Total Records:     339,325
Non‐Orphan:        34,790
Survey:            24,909 (all where data are from a details record from survey)
Survey:            23,585 (all with period from a details record from survey)
Survey:            340 (orphan data, e.g., taxonomic class, from a details record from survey)
U  1‐:            31,429 (non‐orphan) *
U  2‐:            29,729 (non‐orphan)

*   This is the number of entries in the lc_summary table with a period and was given a U rating.
    This excludes summary lines where no period, amplitude, or U code was given. These "no
    data" lines are included to show that some lightcurve data are available but they were
    insufficient to make even an approximate guess of the period and/or amplitude.

Pole:              4,387 (could be just "retrograde vs. prograde")
Tumblers:          549
Binaries:          474 / 410 (all/U  2–, includes suspected) **
Binaries:          357 / 294 (all/U  2–, 'B' or 'M', i.e., considered confirmed)

** The larger counts for binaries with no U rating restriction include binary or multiple asteroids
discovered by adaptive optics, occultation, or spacecraft.
6.1.1 SUMMARY TABLE: U >= 2‐ ONLY
NEA: 1,714 (total + group 9101)
   Binary: 108 (6.3%, includes 'B', 'M', and '?')
   Pole: 75 (4.4%)

Hungaria: 675/340/335 (total/Nesvorny/group 9102) ***
   Binary: 48 (7.1%, includes 'B', 'M', and '?')
   Pole: 65 (9.6%)

Hilda: 172/28/110/34 (total/Nesvorny 001/group 9107/Nesvorny 002, groups 2022,2027) ***
    Binary: 5 (2.9%, includes 'B', 'M', and '?')
    Pole: 35 (20.3%)

Jupiter Trojans: 363/17/346 (total/Nesvorny 004,005,009/group 9202) ***
    Binary: 5 (1.3%)
    Pole: 26 (7.2%)
*** Total based on orbital parameters for group in Table 3)


6.1.2 SUMMARY TABLE: U >= 2‐ ONLY; MIN/MAX VALUES
Shortest Period: 0.003298 h (11.87 s); 2017 QG18
Longest Period: 1880 h (78.33 days); (162058) 1997 AE12
Smallest Diameter: 0.003 km; 2006 RH120, 2010 WA, 2915 TC25
Largest Diameter: 2454 km; (136199) Eris

Based on AmpMax value
Largest Amplitude: 2.80 mag; (24878) 1996 HP25
Average Amp: 0.43. mag
Total: 27,941
Amp 0.01‐0.10:        850      (414 Survey, 48.7%)
Amp 0.11‐0.20:      4,635      (2,998 Survey, 64.7%)
Amp 0.21‐0.30:      5,443      (3,920 Survey, 72.0%)
Amp 0.31‐0.40:      4,294      (3,026 Survey, 70.5%)
Amp 0.41‐0.50:      3,480      (2,621 Survey, 75.3%)
Amp 0.51‐0.75:      6,068      (4,747 Survey, 78.2%)
Amp 0.76‐1.00:      2,411      (1,726 Survey, 71.6%)
Amp 1.01‐1.50:        680      (306 Survey, 45.0%)
Amp > 1.50:            80      (22 Survey, 27.5%)
6.2.0 DETAILS TABLE ‐ OVERVIEW
The numbers for the tumbler and binary subsets are going to be higher than reflected in the
Summary table totals. This just indicates that not every suspected binary or tumbler was "good
enough" to make it to the summary line.

All numbers in each subset include multiple entries for a given asteroid.

Total Records:    780,428           (includes those w/o any LC data)

With Period:       55,398          (includes those w/o U rating)
U  1–:            47,173          (Total: 85.2%. Survey: 29,784, 63.1%)
U  2–:            46,528          (Total: 84.0%. Survey: 28,382, 61.0%)

Pole: 3,944 / 3,609 / 3,058
The first number is the records with a spin axis solution. The second number is those with a period
The third number is those with a period and U >= 2‐

Tumblers: 706 / 510 / 498
The first number is the records that include 'T' in the notes field. The second number is those
with a period (precession and/or rotation). The third number is those with a period and U >= 2–.

Binaries:
Total:         1,036   (includes 'B', 'M', and '?')
Confirmed:       744   (includes 'B', 'M')
7.0     REFERENCES
AstDys (2021). AstDys‐2 web site. https://newton.spacedys.com/astdys2/

Bottke, W.F., Vokrouhlicky, D., Nesvorny, D. (2007). Nature 449, 48‐53; personal
communications.

Bowell, E., Hapke, B., Domingue, D., Lumme, K., Peltoniemi, J., Harris, A.W. (1989). In: Gehrels,
T., Matthews, M.T., Binzel, R.P. (Eds.), Asteroids II, Chapter “Application of Photometric Models
to Asteroids.” University of Arizona Press, pp. 524–555.

Bowell, E., Oszkiewicz, D.A., Wasserman, L.H., Muinonen, K., Penttila, A., Trilling, D.E. (2014).
"Asteroid spin axis longitudes from the Lowell Observatory database." Meteoritics & Planetary
Sci. 49, 95‐102.

Bus, S.J., Binzel, R.P. (2002a). "Phase II of the Small Main‐Belt Asteroid Spectroscopic Survey. The
Observations." Icarus 158, 106‐145.

Bus, S.J., Binzel, R.P. (2002b). "Phase II of the Small Main‐Belt Asteroid Spectroscopic Survey. A
Feature‐Based Taxonomy." Icarus 158, 146‐177.

Chang, C.‐K., Ip, W.‐H., Lin, H.‐W., Cheng, Y.‐C., Ngeow, C.‐C., Yang, T.‐C., Waszczak, A., Kulkarni,
S.R., Levitan, D., Sesar, B., Laher, R., Surace, J., Prince, T.A. (2014). "313 New Asteroid Rotation
Periods from Palomar Transient Factory Observations." Ap. J. 788, A17.

Chang, C.‐K., Ip, W.‐H., Lin, H.‐W., Cheng, Y.‐C., Ngeow, C.‐C., Yang, T.‐C., Waszczak, A., Kulkarni,
S.R., Levitan, D., Sesar, B., Laher, R., Surace, J., Prince, T.A. (2015). “Asteroid Spin‐rate Study Using
the Intermediate Palomar Transient Factory.” Ap. J. Suppl. Ser. 219, A27.

Dermawan, B.; Nakamura, T.; Yoshida, F. (2011). “Subaru Lightcurve Observations of Sub‐km‐
Sized Main‐Belt Asteroids.” Pub. Astron. Soc. Japan 63, S555‐S576. Also Dermawan, B. (2004) PhD
Thesis, School of Science, University of Tokyo, pp 118.

Durech, J., Sidorin, V., Kaasalainen, M. (2010). "DAMIT: a database of asteroid models." Astron.
Astrophys. 513, A46.

Ďurech, J.; Hanuš, J.; Oszkiewsicz, D.; Vanco, R. (2016). “Asteroid models from the Lowell
photometric database.” Astron. Astrophys. 587, A48.

Fukugita, M.; Ichikawa, T.; Gunn, J.E.; Doi, M.; Shimasaku, K.; Schneider, D.P. (1996). “The Sloan
Digital Sky Survey Photometric System.’ Aston. J. 111, 1748.

Harris, A.W., Young, J.W., Scaltriti, F., Zappala, V. (1984). “Lightcurves and phase relations of the
asteroids 82 Alkmene and 444 Gyptis.” Icarus 57, 251‐258.
Hanus, J., Durech, J., Oszkiewicz, D.A., Behrend, R., Carry, B., Delbo, M., Adam, O., Afonina, V.,
Anquetin, R., Antonini, P., and 159 coauthors. (2016). "New and updated convex shape models
of asteroids based on optical data from a large collaboration network." Astron. Astrophys. 586,
A108.

Harris, A.W., Harris, A.W. (1997). “On the Revision of Radiometric Albedos and Diameters of
Asteroids.” Icarus 126, 450‐454.

Harris, A.W., Young, J.W., Scaltriti, F., Zappala, V. (1984). "Lightcurves and phase relations of the
asteroids 82 Alkmene and 444 Gyptis." Icarus 57, 251‐258.

Harris, A.W.; Harris, A.W. (1997). "On the Revision of Radiometric Albedos and Diameters of
Asteroids." Icarus 126, 450‐454.

Harris, A.W., Pravec, P., Warner, B.D. (2012) "Looking a gift horse in the mouth: Evaluation of
wide‐field asteroid photometric surveys." Icarus 221, 226‐235.

Kryszczynska A., La Spina A., Paolicchi P., Harris A.W., Breiter S., Pravec P. (2007). "New findings
on asteroid spin‐vector distributions." Icarus 192, 223‐237.

Mainzer, A., Grav, T., Bauer, J., Masiero, J., McMillan, R.S., Cutri, R.M., Walker, R., Wright, E.,
Eisenhardt, P., Tholen, D.J., Spahr, T., Jedicke, R., Denneau, L., DeBaun, E., Elsbury, D., Gautier,
T., Gomillion, S., Hand, E., Mo, W., Watikins, J., Wilkins, A., Bryngelson, G.L., Del Pino Molia, A.,
Desai, S., Gomez Camus, M., Hidalgo, S.L., Konstantopoulos, I., Larsen, J.A., Maleszewski, C.,
Malkan, M.A., Mauduit, J.‐C., Mullan, B.L., Olszewski, E.W., Pforr, J., Saro, A., Scotti, J.V.,
Wasserman, L.H. (2011). "NEOWISE observations of near‐Earth objects: Preliminary results." Ap.
J. 743, A156.

Mahlke, M.; Carry, B.; Denneau, L. (2021). “Asteroid phase curves from ATLAS dual‐band
photometry.” Icarus 354, id. 114094.

Mainzer, A., Bauer, J., Cutri, R., Grav, T., Kramer, E., Masiero, J., Sonnett, S., Wright, E., Eds.
(2019). “NEOWISE Diameters and Albedos V2.0.” NASA Planetary Data System.
urn:nasa:pds:neowise_diameters_albedos::2.0.
https://doi.org/10.26033/18S3‐2Z54

Muinonen, K., Belskaya, I.N., Cellino, A., Delbo, M., Levasseur‐Regourd, A.‐C., Pentttila, A.,
Tedesco, E.F. (2010). “A three‐parameter magnitude phase function for asteroids.” Icarus 209,
542–555.

Nesvorny, D., Broz, M., Carruba, V. (2015). “Identification and Dynamical Properties of Asteroid
Families.” In Asteroids IV (P. Michel, F. DeMeo, W.F. Bottke, R. Binzel, Eds.). Univ. of Arizona Press,
Tucson, also available on astro‐ph.

Nesvorny, D. (2015). “Nesvorny HCM Asteroids Families V3.0.” NASA Planetary Data Systems, id.
EAR‐A‐VARGBET‐5‐NESVORNYFAM‐V3.0.
Polishook, D., Ofek, E. O., Waszczak, A., Kulkarni, S. R., Gal‐Yam, A., Aharonson, O., Laher, R.,
Surace, J., Klein, C., Bloom, J., Brosch, N., Prialnik, D., Grillmair, C., Cenko, S. B., Kasliwal, M., Law,
N., Levitan, D., Nugent, P., Poznanski, D., Quimby, R. (2012). "Asteroid rotation periods from the
Palomar Transient Factory survey." Mon. Not. Royal Ast. Soc. 421, 2091‐2108.

Pravec, P., Harris, A.W., Scheirich, P., Kušnirák, P., Šarounová, L., Hergenrother, C.W., Mottola,
S., Hicks, M.D., Masi, G., Krugly, Yu.N., Shevchenko, V.G., Nolan, M.C., Howell, E.S., Kaasalainen,
M., Galád, A., Brown, P., Degraff, D.R., Lambert, J. V., Cooney, W.R., Foglia, S. (2005). "Tumbling
asteroids." Icarus 173, 108‐131.

Pravec, P., Scheirich, P., Kusnirák, P., Sarounová, L., Mottola, S., Hahn, G., Brown, P., Esquerdo,
G., Kaiser, N., Krzeminski, Z., and 47 coauthors (2006). "Photometric survey of binary near‐Earth
asteroids." Icarus 181, 63‐93.

Pravec, P., Vokrouhlicky, D., Polishook, D., Scheeres, D.J., Harris, A.W., Galad, A., Vaduvescu, O.,
Pozo, F., Barr, A., Longa, P., and 16 coauthors. (2010). "Formation of asteroid pairs by rotational
fission," Nature 466, 1085‐1088.

Rodgers, C.T.; Canterna, R.; Smith, J.A.; Pierce, M.J.; Tucker, D.L. (2006). “Improved u´g´r´i´z´ to
UBVRCIC Transformation Equations for Main‐Sequence Stars.” Astron. J. 132, 989‐993.

Shevchenko, V.G.; Belsakya, I.N.; Muinonen, K.; Penttila, A.; Krugly, Y.N.; Velichko, F.P.; Chiorny,
V.G.; Slyusarev, I.G.; Gaftonyuk, N.M.; Tereschenko, I.A. (2016). “Asteroid observations at low
phase angles. IV. Average parameters for the new H, G1, G2 magnitude system.” Planet. Space
Sci. 123, 101‐116.

Tedesco, E.F., Noah, P.V., Price, S.D. (2004). "IRAS Minor Planet Survey." IRAS‐A‐FPA‐3‐RDR‐IMPS‐
V6.0. NASA Planetary Data System.

Tholen, D.J. (1984). “Asteroid taxonomy from cluster analysis of Photometry.” Doctoral Thesis.
Univ. Arizona, Tucson.

Usui, F., Kuroda, D., Müller, T.G., Hasegawa, S., Ishiguro, M., Ootsubo, T., Ishihara, D., Kataza, H.,
Takita, S., Oyabu, S., Ueno, M., Matsuhara, H., Onaka, T. (2011). "Asteroid Catalog Using Akari:
AKARI/IRC Mid‐Infrared Asteroid Survey." Pub. Ast. Soc. of Japan 63, 1117‐1138.

Veres, P., Jedicke, R., Fitzsimmons, A., Denneau, L., Granvik, M., Bolin, B., Chastel, S., Wainscoat,
R.J., Burgett, W.S., Chambers, K.C., Flewelling, H., Kaiser, N., Magnier, E.A., Morgan, J.S., Price,
P.A., Tonry, J.L., Waters, C. (2015). "Absolute magnitudes and slope parameters for 250,000
asteroids observed by Pan‐STARRS PS1 ‐ Preliminary results." Icarus 241, 34‐47.

Warner, B.D., Harris, A.W., Pravec, P. (2009). "The Asteroid Lightcurve Database." Icarus 202,
134‐146. Updates at http://www.minorplanet.info/lightcurvedatabase.html

Warner, B.D., Harris, A.W. (2011) "Using sparse photometric data sets for asteroid lightcurve
studies." Icarus 216, 610‐624.
Waszczak, A., Chang, C.‐K., Ofek, E.O., Laher, R., Masci, F., Levitan, D., Surace, J., Cheng, Y.‐C., Ip,
W.‐H., Kinoshita, D., Helou, G., Prince, T.A., Kulkarni, S. (2015). "Asteroid Light Curves from the
Palomar Transient Factory Survey: Rotation Periods and Phase Functions from Sparse
Photometry." Astron. J. 150, A75.
