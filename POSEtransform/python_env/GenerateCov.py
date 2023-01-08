import numpy as np

# data is a 2D matrix with shape (n, 3), where n is the number of observations
data = [[ 0.3255419335292595 , 0.5561627687116518 , 0.8064846316280417 ] ,
[ -0.1387768495026866 , 0.0805536389598237 , 1.6717985266138573 ] ,
[ 0.06781518102253883 , 0.06359466812067487 , 2.6211469891751804 ] ,
[ -0.5848951596165595 , 0.18683693952823177 , 4.540629504855863 ] ,
[ -0.5932059254244471 , 0.1536788937004989 , 4.967237825730176 ] ,
[ -0.054603500508009106 , 0.07494556533774821 , 5.811369687959274 ] ,
[ -0.2788105135196689 , 0.01666308252722204 , 6.511286818491814 ] ,
[ 0.5486364373785032 , 0.41172483232276325 , 8.434611634812251 ] ,
[ -0.653454362813022 , -0.33336029754964 , 9.206919704775201 ] ,
[ -0.06335922172144726 , 0.09169741656761943 , 10.306374850128451 ] ,
[ -0.15255706737304525 , 0.12231595259223688 , 10.560152867872555 ] ,
[ 0.10658703085769661 , 0.6312671150276681 , 11.716696176211816 ] ,
[ -0.7189121761529892 , 0.1896421399041356 , 13.691006207162179 ] ,
[ -0.6907386766214749 , 0.3054992623144265 , 14.42677055206492 ] ,
[ -0.04971961933518443 , -0.18223903764366134 , 15.225112775444364 ] ,
[ -0.6265200797340001 , 0.38212681543636057 , 16.169093705840957 ] ,
[ 0.47643732876629097 , 0.34634060111293297 , 17.614837891937203 ] ,
[ -0.13493861307390154 , 0.5412677310598649 , 18.10971089383099 ] ,
[ -0.31709530870511393 , -0.2561143817401389 , 19.599727196866375 ] ,
[ -0.8680720849993726 , 0.10138841539221921 , 20.170097954092324 ] ,
[ -0.983003691580123 , 0.5060627520481221 , 21.727644694072282 ] ,
[ -0.8845892964601202 , 0.538260670238855 , 22.114970190758637 ] ,
[ 0.13327816683084734 , 0.4160520245801315 , 23.2603587996777 ] ,
[ 0.08535745219846047 , 0.522096297760174 , 23.633061910382402 ] ,
[ 0.08298863425730371 , 0.29440150114692853 , 24.572130233297283 ] ,
[ 0.10960903031770787 , 0.3438611796669657 , 25.99743552135599 ] ,
[ -0.6779236433535902 , 0.5059288724617749 , 26.70215048409582 ] ,
[ -0.47426360984273586 , 0.2897700984689098 , 27.829801985750915 ] ,
[ -0.6145504015277082 , 0.18676636394621904 , 28.926227333663046 ] ,
[ 0.15042019339807922 , 0.33295527582820217 , 30.268736869867098 ] ,
[ -0.47917311953238684 , 0.3643723909018315 , 31.39217861576022 ] ,
[ -0.658096055155162 , 0.09481747628004661 , 31.653983610426824 ] ,
[ 0.002744841560473388 , 0.048986141768089186 , 32.72112701420029 ] ,
[ -0.7298296968659103 , -0.29198867315602983 , 34.798662307963205 ] ,
[ -0.2051176578213214 , -0.11442120287624033 , 34.97952444982165 ] ,
[ -1.0813554894111026 , -0.019760854599037724 , 36.401038410841444 ] ,
[ -0.548247831320535 , 0.19412408769073053 , 37.359470199028635 ] ,
[ 0.27600257882358453 , 0.044727673258736 , 37.752554554801115 ] ,
[ -0.8014307524478579 , 0.29225530272695843 , 38.83608136954194 ] ,
[ 0.3386200265252524 , 0.532202924457492 , 39.508608750242885 ] ,
[ -0.5293803369858481 , 0.27562124740020644 , 40.963129506550544 ] ,
[ -0.639610965354436 , 0.07604303744550245 , 42.20003547189557 ] ,
[ 0.47509454365782333 , -0.07659471861358186 , 43.1069444562024 ] ,
[ 0.20795230410947543 , -0.12615358243198083 , 44.448608496379926 ] ,
[ -0.11388156956200435 , 0.09168895678535843 , 44.97316722953053 ] ,
[ 0.45981384375838374 , 0.45072312443743423 , 46.43683511052384 ] ,
[ -0.6135675949290359 , 0.5267141201645005 , 47.25986974779289 ] ,
[ -0.051333262061764806 , 0.1961963940463648 , 48.3847397024158 ] ,
[ 0.1427295386602765 , -0.0001770556473895124 , 49.23300603605918 ] ,
[ 0.22197886975959158 , 0.11893533876337259 , 50.312190079560104 ] ,
[ -0.1917417412204554 , -0.23493105277900533 , 51.24311701881722 ] ,
[ -0.5708911686110067 , -0.04510876779210049 , 52.349522831615275 ] ,
[ -0.26335537156113187 , 0.3761952738042832 , 53.110418614841734 ] ,
[ -0.1417206233255991 , -0.20489642366159716 , 54.00258328690228 ] ,
[ -0.10223547705868719 , -0.04681837247351761 , 55.591941912174725 ] ,
[ -0.07780827069438312 , 0.053366747967822814 , 56.24672834800211 ] ,
[ -0.6417765737031991 , 0.4018501780964202 , 57.23765997306697 ] ,
[ -0.11843348004670184 , 0.6269695257779275 , 57.4749169904257 ] ,
[ -0.8645081592130994 , -0.20031179248601794 , 58.81583177955007 ] ,
[ 0.6101320008726397 , -0.07094273148547955 , 60.11611456690608 ] ,
[ -0.33638126057429396 , 0.40058168960910545 , 60.87206567716081 ] ,
[ -0.196450644520888 , 0.21799577642169843 , 62.2790887372098 ] ,
[ -0.393803159029891 , 0.5060454930174001 , 63.18430316785057 ] ,
[ -0.020943134874158886 , 0.2077592294676084 , 64.22560923164941 ] ,
[ -0.15116390873231256 , 0.008960323990225005 , 65.35806595774037 ] ,
[ 0.1552783279149893 , -0.1404320282243752 , 66.48027377342113 ] ,
[ -0.6755469239127844 , 0.4383386064987541 , 67.27948595362369 ] ,
[ -0.04296928465611671 , 0.21366300980471709 , 67.93942852832589 ] ,
[ -0.4383507359054812 , 0.1688638726540369 , 68.59275696358458 ] ,
[ -0.09137643924749303 , 0.25796415815743134 , 70.6123762355294 ] ,
[ 0.5334702176247819 , 0.7121756712272252 , 71.48401218743852 ] ,
[ -0.646763662661459 , -0.2434266279904961 , 71.84386371957342 ] ,
[ -0.1649631828751296 , 0.3255868255991434 , 73.25131338598398 ] ,
[ 0.05510054317217089 , 0.6366667993465018 , 74.11695330498341 ] ,
[ 0.5454409290867793 , 0.39112691079432105 , 75.40047599528282 ] ,
[ -0.2882401117406145 , 0.553395294524438 , 75.6803650661903 ] ,
[ 0.44985637831703723 , -0.10834406121477014 , 77.09734748904067 ] ,
[ 0.5985271529055421 , 0.33698451098188525 , 77.99056538314112 ] ,
[ -0.42351417669971714 , -0.03049175319986397 , 79.67371325705436 ] ,
[ 0.1319854346337188 , -0.019872377014140574 , 80.16742179649933 ] ,
[ -0.5062008618079015 , 0.17645520608042342 , 80.85529064585904 ] ,
[ 0.013298078125330515 , 0.6201414436865103 , 82.4819275573834 ] ,
[ 0.13788483110047284 , 0.21958825463865272 , 83.149354789596 ] ,
[ -0.7426354396178265 , 0.059121095102790844 , 83.51125931814755 ] ,
[ 0.3090440078072655 , 0.45745336269917747 , 84.51591477974344 ] ,
[ -0.22464572038810693 , 0.6652477300834645 , 85.57542654336454 ] ,
[ 0.5598160055220222 , -0.10245914674041712 , 86.68155745349415 ] ,
[ -0.630903580620476 , 0.3627346462930121 , 87.72257831482054 ] ,
[ 0.205554206033883 , 0.5898379917387914 , 89.58999613057962 ] ,
[ -0.4303039467233831 , -0.1285605744262759 , 90.23674179080555 ] ,
[ -0.24508230251512927 , 0.3784303563016701 , 91.10103632976782 ] ,
[ -0.4410391770651029 , 0.0016927728445957912 , 92.30707468647547 ] ,
[ -0.7461140932892438 , 0.16199137316120882 , 93.48406459120119 ] ,
[ 0.0591761343593491 , 0.5432555050275761 , 93.95157773839233 ] ,
[ -0.07797848297006771 , 0.09261604843625347 , 94.8024720927503 ] ,
[ -0.4226784508611827 , -0.3160535640857144 , 95.56213658400746 ] ,
[ -0.006417233653843835 , -0.26331069645168514 , 96.83819790717796 ] ,
[ -0.5669416592139166 , 0.3318245418597879 , 98.7218077185484 ] ,
[ -0.6865565154535652 , 0.2418249108508218 , 98.57771939917271 ] ,
[ -0.4964816956924788 , -0.15450038241618846 , 100.77546217015814 ]]


gt = [[ 0 , 0 , 1 ] ,
[ 0 , 0 , 2 ] ,
[ 0 , 0 , 3 ] ,
[ 0 , 0 , 4 ] ,
[ 0 , 0 , 5 ] ,
[ 0 , 0 , 6 ] ,
[ 0 , 0 , 7 ] ,
[ 0 , 0 , 8 ] ,
[ 0 , 0 , 9 ] ,
[ 0 , 0 , 10 ] ,
[ 0 , 0 , 11 ] ,
[ 0 , 0 , 12 ] ,
[ 0 , 0 , 13 ] ,
[ 0 , 0 , 14 ] ,
[ 0 , 0 , 15 ] ,
[ 0 , 0 , 16 ] ,
[ 0 , 0 , 17 ] ,
[ 0 , 0 , 18 ] ,
[ 0 , 0 , 19 ] ,
[ 0 , 0 , 20 ] ,
[ 0 , 0 , 21 ] ,
[ 0 , 0 , 22 ] ,
[ 0 , 0 , 23 ] ,
[ 0 , 0 , 24 ] ,
[ 0 , 0 , 25 ] ,
[ 0 , 0 , 26 ] ,
[ 0 , 0 , 27 ] ,
[ 0 , 0 , 28 ] ,
[ 0 , 0 , 29 ] ,
[ 0 , 0 , 30 ] ,
[ 0 , 0 , 31 ] ,
[ 0 , 0 , 32 ] ,
[ 0 , 0 , 33 ] ,
[ 0 , 0 , 34 ] ,
[ 0 , 0 , 35 ] ,
[ 0 , 0 , 36 ] ,
[ 0 , 0 , 37 ] ,
[ 0 , 0 , 38 ] ,
[ 0 , 0 , 39 ] ,
[ 0 , 0 , 40 ] ,
[ 0 , 0 , 41 ] ,
[ 0 , 0 , 42 ] ,
[ 0 , 0 , 43 ] ,
[ 0 , 0 , 44 ] ,
[ 0 , 0 , 45 ] ,
[ 0 , 0 , 46 ] ,
[ 0 , 0 , 47 ] ,
[ 0 , 0 , 48 ] ,
[ 0 , 0 , 49 ] ,
[ 0 , 0 , 50 ] ,
[ 0 , 0 , 51 ] ,
[ 0 , 0 , 52 ] ,
[ 0 , 0 , 53 ] ,
[ 0 , 0 , 54 ] ,
[ 0 , 0 , 55 ] ,
[ 0 , 0 , 56 ] ,
[ 0 , 0 , 57 ] ,
[ 0 , 0 , 58 ] ,
[ 0 , 0 , 59 ] ,
[ 0 , 0 , 60 ] ,
[ 0 , 0 , 61 ] ,
[ 0 , 0 , 62 ] ,
[ 0 , 0 , 63 ] ,
[ 0 , 0 , 64 ] ,
[ 0 , 0 , 65 ] ,
[ 0 , 0 , 66 ] ,
[ 0 , 0 , 67 ] ,
[ 0 , 0 , 68 ] ,
[ 0 , 0 , 69 ] ,
[ 0 , 0 , 70 ] ,
[ 0 , 0 , 71 ] ,
[ 0 , 0 , 72 ] ,
[ 0 , 0 , 73 ] ,
[ 0 , 0 , 74 ] ,
[ 0 , 0 , 75 ] ,
[ 0 , 0 , 76 ] ,
[ 0 , 0 , 77 ] ,
[ 0 , 0 , 78 ] ,
[ 0 , 0 , 79 ] ,
[ 0 , 0 , 80 ] ,
[ 0 , 0 , 81 ] ,
[ 0 , 0 , 82 ] ,
[ 0 , 0 , 83 ] ,
[ 0 , 0 , 84 ] ,
[ 0 , 0 , 85 ] ,
[ 0 , 0 , 86 ] ,
[ 0 , 0 , 87 ] ,
[ 0 , 0 , 88 ] ,
[ 0 , 0 , 89 ] ,
[ 0 , 0 , 90 ] ,
[ 0 , 0 , 91 ] ,
[ 0 , 0 , 92 ] ,
[ 0 , 0 , 93 ] ,
[ 0 , 0 , 94 ] ,
[ 0 , 0 , 95 ] ,
[ 0 , 0 , 96 ] ,
[ 0 , 0 , 97 ] ,
[ 0 , 0 , 98 ] ,
[ 0 , 0 , 99 ] ,
[ 0 , 0 , 100 ]]
# Convert the data to a numpy array
data = np.array(data)



# Center the data by subtracting the mean
centered_data = data - gt

print(np.max(centered_data,axis = 0))
# Calculate the mean of each column
mean = np.mean(centered_data**2, axis=0)
print(np.sqrt(mean))

# Calculate the covariance matrix
covariance_matrix = np.cov(centered_data, rowvar=False)

print(covariance_matrix)