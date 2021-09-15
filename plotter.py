import matplotlib.pyplot as plt

#plotter
x_F1 = [0, 4.971894083465621, 9.941761601573608, 14.923809251280675, 19.932602512173865, 24.922198202195894, 29.916269104355077, 35.01776457743929, 40.032919197088816, 45.11039266712192, 50.11305819868714, 55.23725400017344, 60.28084538866442, 65.45721682302805, 70.5035012700917, 75.60069322328344, 80.49349601888375, 85.18669290593554, 89.4975491289685, 93.55366456324461, 97.66706046233418, 101.94677589346047, 106.43080756674487, 111.19088874908559, 116.25996707434054, 121.45173171296634, 126.58137159674779, 131.64722278605194, 136.57356446177016, 141.44132724977388, 146.3157670670251, 151.21325106372203, 156.0931867014446, 160.84591830416895, 165.42302081955128, 169.80159823113567, 174.1456980014186, 178.62953186635983, 183.23631959837957, 188.02727644763965, 192.74435870230647, 197.56427721522365, 202.47335532335637, 207.77965247728307, 212.8453352851984, 218.00852905664937, 222.999675847001, 228.0508010901192, 233.06897420049373, 238.0736783353319, 243.1079053864975, 248.09688998818342, 253.09213416389082, 258.1126525647987, 263.1097744700225, 268.1056411582419, 273.10294177810005, 278.1142526249942, 283.1284937221141, 288.13064734820654, 293.1334349263017, 298.1468172976594, 303.1619828091535, 308.16498371600807, 313.16804797081124, 318.1797750570473, 323.19292807707006, 328.19602137845095, 333.19919072834824, 338.207975630963, 343.21869721524286, 348.22171804832874, 353.2246885430613, 358.2301559538446, 363.2363786048228, 368.2401099611619, 373.24302251475063, 378.2456061622259, 383.24791916451767, 388.24999935102005]
vel_F1 = [83.        , 83. ,        83.,         83.      ,   83.     ,    82.23785521,
 80.39540146, 78.46866156, 76.52726031, 74.51022186 ,72.46801522, 70.31473162,
 68.12889254, 65.81005784, 63.46799702, 61.0111066  ,58.55584481, 56.09986763,
 53.74517443, 51.43128357, 48.97318833, 46.27731896 ,43.272997  , 39.83650836,
 35.81626967, 31.16657874, 30.49623933, 32.89365688 ,35.06826076, 37.091991,
 39.0134449 , 40.85307092, 42.6071763 , 44.24876745 ,45.77408105, 47.18711496,
 48.54839659, 49.91454525 ,51.28026384, 52.66302465 ,53.98987528, 55.31278503,
 56.62840746, 58.01694102, 59.3121904 , 60.60389214 ,61.82690472, 63.04046244,
 64.22339992, 65.38184933, 66.52678433, 67.64231347, 68.74110776, 69.82804199,
 70.89336433 ,71.94265151, 72.97714796, 74.00002331, 75.00953728, 76.00325843,
 76.98427709, 77.95498951, 78.91410112, 79.85941011, 80.79367123, 81.71884191,
 82.63391382, 83.  ,       83.        , 83. ,        83.        , 83.,
 83.         ,83. ,        83.         ,83.,         83.        , 83.,
 83.         ,83.     ]
curvature_F1 = [4.95113461e-03, 2.06167375e-03, 4.11394622e-03, 5.75281081e-03,
 3.58794246e-03, 5.90211605e-03, 5.94225847e-03, 6.17625979e-03,
 3.08258412e-03, 7.07567464e-03 ,7.28810374e-03, 8.08073245e-03,
 8.61669810e-03, 9.21462443e-03, 9.92530262e-03, 1.07458747e-02,
 1.16659319e-02, 1.27056144e-02, 1.38478081e-02, 1.51218288e-02,
 1.66778586e-02, 1.86777092e-02 ,2.13611849e-02, 2.52056244e-02,
 3.11816641e-02 ,4.11795626e-02, 4.30097994e-02, 3.69688274e-02,
 3.25259849e-02, 2.90736079e-02, 2.62803649e-02, 2.39666641e-02,
 2.20335713e-02, 2.04280852e-02, 1.90905243e-02, 1.79636043e-02,
 1.69703980e-02, 1.57715643e-02, 1.52106488e-02, 1.43480473e-02,
 1.28831583e-02 ,1.30727298e-02, 1.24732183e-02, 1.18808103e-02,
 1.13695542e-02, 1.08907569e-02 ,1.04603432e-02, 9.52236347e-03,
 9.10002745e-03, 8.30731974e-03, 8.37365338e-03, 8.64024910e-03,
 8.32031280e-03, 2.07112978e-03, 7.77332309e-03, 1.96166600e-03,
 7.33665671e-03, 2.03764344e-04 ,7.03700156e-03, 5.98334475e-04,
 6.66942934e-03, 5.68029184e-04, 6.37760171e-03, 6.17959922e-04,
 5.87364671e-03 ,2.16299557e-04, 5.81502700e-03, 5.38389158e-04,
 4.94586764e-03, 9.94224875e-05 ,5.18551553e-03, 1.01623786e-03,
 3.54404533e-03, 7.75451224e-04, 5.03014383e-03, 1.78450934e-03,
 9.99881616e-04, 7.10210938e-04, 5.19330978e-04, 3.48010827e-04]


x_s = [0, 4.997115351013809, 9.997498828444495, 15.000998504641359, 20.003533572871948, 25.014982819365983, 30.04339622775784, 35.057829642775914, 40.06155472911353, 45.07735384836832, 50.084170155088984, 55.08258297029572, 60.10837454083175, 65.12257384405633, 70.14761054692784, 75.1530734824848, 80.1228137862243, 84.97521384474253, 89.67666709536945, 94.29034494143575, 99.11088757656708, 104.03165999681963, 109.24761061724178, 114.27428977960706, 119.31564118823844, 124.34986191916833, 129.43293686340823, 134.4006707133581, 139.2193809073026, 144.09768402023965, 149.02896337575015, 154.11350842069166, 159.16488077999907, 164.14341639211483, 169.03020415912405, 173.72572914903353, 178.35705972046242, 183.03232678812194, 187.77871477515114, 192.61086078489816, 197.4089003179504, 202.18358017196746, 207.11766377462865, 212.24600408851956, 217.33686180111093, 222.38311080790464, 227.39417159715615, 232.38916320340599, 237.38090926653834, 242.3819409708221, 247.39504479695898, 252.442955534412, 257.48900323448663, 262.5011439059653, 267.52981169862875, 272.53207747288894, 277.55040885272695, 282.5555954340869, 287.5570754590914, 292.55903396212733, 297.56049829717597, 302.5623914147873, 307.5652573800999, 312.5675426386909, 317.57049562184596, 322.5751834613883, 327.57771260489295, 332.58035046965153, 337.58309619501415, 342.585846921722, 347.58866094682566, 352.59150685338983, 357.5942911263683, 362.60224533044305, 367.6062263199015, 372.6100154337591, 377.6137725360062, 382.6161995531029, 387.61849284514386, 392.6208932711546]
vel_s = [58.68059308, 57.9212525  ,56.61148282, 55.26983803, 53.89507347, 52.48176318,
 51.02433796, 49.52827538, 47.98893945, 46.39465849, 44.74661827 ,43.03844168,
 41.24965109, 39.38410512, 37.4213393 , 35.35957281, 33.18444184 ,30.91334952,
 28.54105081 ,26.00348526, 23.05569272, 19.59443273, 16.80118738 ,18.9830488,
 20.99316574 ,22.83113093, 24.54671819, 26.10466114, 27.52816839 ,25.41852224,
 22.51541095, 19.04203979, 19.98948327 ,21.88578842, 23.5474263  ,25.0682118,
 26.50502843, 27.87931575, 29.20786201 ,30.50268086, 31.73613354 ,32.91773154,
 34.09573762 ,35.27850293, 36.41340453, 37.50501562, 38.55908672 ,39.58142498,
 40.57779122, 41.55205948, 42.50627376, 43.44594205, 44.36537663 ,45.26014687,
 46.14043323, 46.99974289, 47.84630742, 48.67599119, 49.4911689  ,50.29321162,
 51.08258573, 51.86001209, 52.62610293, 53.38111346, 54.12569188 ,54.86041858,
 55.58512385, 56.30051687, 57.00694809, 57.70473241, 58.39418778 ,59.07560158,
 59.7492364 , 60.4160452,  61.07505393, 61.72700251, 62.37213282 ,63.01048948,
 63.64242671 ,64.26816385]
curvature_s = [3.77532517e-03, 6.15255242e-04, 3.59093462e-03, 4.14645092e-03,
 3.17112919e-03, 4.42751638e-03, 4.96896509e-03, 5.04063182e-03,
 1.39354365e-03, 6.02614296e-03, 6.48745218e-03, 6.98332374e-03,
 7.63320317e-03, 8.36823197e-03, 9.28334945e-03, 1.03780838e-02,
 1.17971120e-02, 1.35859378e-02, 1.59305789e-02, 1.91711483e-02,
 2.44379094e-02, 3.38592990e-02, 4.60535806e-02, 3.60754223e-02,
 2.94976544e-02, 2.49179124e-02 ,2.15752819e-02, 1.90768749e-02,
 1.71549209e-02, 2.01206847e-02, 2.56438718e-02 ,3.58522500e-02,
 3.25342063e-02, 2.71405697e-02, 2.34453359e-02, 2.06869584e-02,
 1.84934329e-02, 1.67255008e-02, 1.52385560e-02, 1.39703347e-02,
 1.29061205e-02, 1.19961732e-02, 1.11826098e-02, 1.04445620e-02,
 9.80439457e-03, 9.24197206e-03, 8.74279177e-03, 8.29775298e-03,
 7.89496818e-03, 7.52892228e-03, 6.06088850e-03, 1.70767922e-03,
 5.75864311e-03, 6.34547923e-03, 5.83628591e-03, 2.37861373e-03,
 5.62594883e-03, 1.83179534e-03, 3.04467021e-03, 5.63462615e-05,
 1.19871825e-03, 1.78424939e-03, 9.88195325e-04, 8.16421600e-04,
 1.85859478e-03, 6.19393279e-04, 1.67511311e-03, 3.76977563e-04,
 2.34941556e-04, 1.37280098e-04, 3.97967974e-04, 2.32596736e-03,
 3.57834315e-03, 1.27045503e-04, 3.44094655e-03, 3.37640498e-03,
 1.85142703e-04, 5.08547836e-04, 8.60966374e-04, 8.07961492e-04]

x10 = [0, 5.013383544112252, 10.016402638348398, 15.01997090124416, 20.024828339046696, 25.02366493278479, 30.02542088997698, 35.019152079080655, 40.010494399106946, 45.00322422954069, 50.00249452338277, 55.019797785641266, 60.08279171580534, 65.18447590838657, 70.29529792944084, 75.36374408791052, 80.356725218763, 85.20199677203581, 89.89363504989846, 94.49581255577884, 99.13283238105946, 103.820736932967, 108.52275595761401, 113.14002527360084, 117.6252297057622, 122.10763216441147, 126.8985894842106, 132.10558264633238, 137.4090288597508, 142.56114021721152, 147.59266395595716, 152.62910023802445, 157.7056447381902, 162.75343457728576, 167.7509967851727, 172.71909805463395, 177.65667317349616, 182.570574926138, 187.47816156795628, 192.38916513353033, 197.29118609506068, 202.13734803182, 206.9136389725334]
v10 = [83.  ,       83.    ,     83.,         83.   ,      83.,         83.,
 83.         ,83.         ,82.5224405,  80.68698406, 78.80630164, 76.87258928,
 74.87065745, 72.79776298, 70.66360299, 68.4780112  ,66.25450287 ,64.02338458,
 61.78588412 ,59.50936755 ,57.32562184 ,55.25472714 ,56.09924467, 56.91634773,
 57.69899828, 58.47069737, 59.28441277, 60.15630857, 61.03141897 ,61.86983376,
 62.67780152 ,63.47625957 ,64.27103873 ,65.0516888  ,65.81545001, 66.56602351,
 67.30369223, 68.02988331, 68.74748545, 69.45816602, 70.16036806 ,70.84772745,
 71.51871296]
xcombo = [0, 5.047784724976057, 10.067423881929166, 15.082413995973718, 20.086302195181577, 25.08926873870133, 30.1026923531648, 35.096365758772116, 40.087707750069164, 45.08091308233682, 50.078895009665075, 55.09935274549174, 60.15783592197474, 65.26576298628187, 70.36832222901022, 75.44504243187129, 80.42799943015343, 85.28163089421199, 89.96700584965204, 94.5745466871818, 99.2143347491368, 103.9035528540619, 108.61596482661714, 113.23031600514656, 117.71256396699562, 122.15169755564602, 126.87422514039312, 131.9953383248013, 137.22046013468673, 142.36284351999834, 147.38347915638414, 152.41134791267018, 157.45847439906015, 162.47014186405104, 167.48714019366594, 172.46777315814785, 177.42514813950194, 182.39039558920487, 187.3774217205659, 192.39761105618075, 197.43987918836643, 202.4588212277506, 207.37978576788464, 212.16826447251017, 216.77101863871678, 221.20741815710198, 225.54876350126602, 229.94931772022662, 234.47553734851348, 239.09951734973777, 243.85521492827337, 248.6548731189626, 253.56170301539936, 258.4640077058511, 263.5246635165506, 268.5653373186963, 273.76560262395014, 278.8555409960833, 284.0155846870804, 289.02532834451995, 294.0487466512382, 299.10105544687485, 304.09495147986036, 309.12433857522063, 314.12955167842097, 319.132892974992, 324.1806970307749, 329.2240560648828, 334.23936890742056, 339.2383688523371, 344.2401766312193, 349.30750351508806, 354.30262621984485, 359.2984947072495, 364.29451929034616, 369.29275988920193, 374.2926280415311, 379.29425451784226, 384.2979010522204, 389.30411236281674]
vcombo = [83. ,        83. ,        83.,         83. ,        83.,         83.,
 83.         ,83.         ,82.72255901, 80.89146709 ,79.01614096, 77.08646489,
 75.09203738 ,73.02286255, 70.89565599 ,68.71383285, 66.50273232 ,64.27593265,
 62.05056825 ,59.78148012 ,57.40590633, 54.90068301 ,52.26639879, 53.12356442,
 53.96070844 ,54.77719167, 55.6326458  ,56.54567661, 57.4623005  ,58.35035219,
 59.2045295  ,60.04776173 ,60.88247874, 61.70015857 ,62.50799576, 63.29978036,
 64.07815301 ,63.98909563, 61.60667814 ,59.11151911, 56.49456261 ,53.76336189,
 50.94351152, 48.04094757 ,45.07513054, 45.28184346 ,46.23062101, 47.17287235,
 48.12280414, 49.074269  , 50.03396676 ,50.98422298, 51.93772801 ,52.87318493,
 53.821806  , 54.750345   ,55.69206033, 56.59862499 ,57.50308883, 58.36778901,
 59.22218469, 60.06923785, 60.89491979 ,61.71530603, 62.52106253 ,63.31627031,
 64.10854988, 64.89047194 ,65.65881209, 66.41580839 ,67.16469132, 67.91496372,
 68.64652031, 69.37047009, 70.08696464 ,70.79652128, 71.49926425 ,72.19541064,
 72.88518538, 73.56884173]

x = [0, 4.990415548972057, 9.984802156426078, 14.982920447690612, 19.98477429502056, 24.990003757572534, 29.998396530387943, 35.00972037780959, 40.02376904492621, 45.0405852037341, 50.06058625257958, 55.08151410862083, 60.076411515842, 65.01964508375728, 69.90766319079304, 74.737376558919, 79.49582042759872, 84.15919515388632, 88.72658901554921, 93.26533844305817, 97.87961371537419, 102.55002081286803, 107.28932884230534, 111.92245130251928, 116.3957537414614, 120.7126051644512, 125.15515854990267, 129.91313002160638, 134.95686155654326, 140.16296804594072, 145.3123238138563, 150.36068739189972, 155.30211716423378, 160.20701195724027, 165.12370045346404, 170.0670766628524, 175.04830480410928, 180.0809468449884, 185.1709374619642, 190.3444377378119, 195.60063576433842, 200.8944383071756, 206.14530565871843, 211.25376423184431, 216.1674907765588, 220.8343384838796, 225.33577876156974, 229.8168192527976, 234.35343683977123, 238.97379761345243, 243.66532229896472, 248.4362724288739, 253.23843424314032, 258.1298154769078, 263.07648229291914, 268.168312688979, 273.2747983182837, 278.45605918759185, 283.5509970693336, 288.6149922659916, 293.60260465902974, 298.5902204880967, 303.5991117375669, 308.60395409490667, 313.6021715671734, 318.60974090328756, 323.62280933033935, 328.6342008505151, 333.64316240834677, 338.6488532655069, 343.648357892149, 348.67585825046126, 353.6974946935401, 358.69389037266893, 363.69158111661915, 368.6938316674176, 373.70065893854405, 378.6995957648152, 383.70058034960357, 388.7100177181095, 393.7225886038529, 398.72926349203755, 403.715253430326, 408.62186800588415, 413.4663233909153, 418.24296017170707, 422.9600279090377, 427.6531163954723, 432.3976253176564, 437.21269521552546]

height = [94.86832980505139, 94.33981132056603, 93.8083151964686, 93.27379053088816, 92.73618495495704, 92.19544457292886, 91.6515138991168, 91.10433579144299, 90.55385138137417, 90.0, 89.44271909999159, 88.88194417315589, 88.31760866327848, 87.74964387392123, 87.17797887081348, 86.60254037844388, 86.02325267042627, 85.44003745317531, 84.85281374238569, 84.2614977317636, 83.66600265340756, 83.06623862918075, 82.46211251235322, 81.85352771872451, 81.24038404635961, 80.62257748298549, 80.0, 79.37253933193772, 78.74007874011811, 78.10249675906654, 77.45966692414834, 76.81145747868608, 76.15773105863909, 75.4983443527075, 74.83314773547883, 74.16198487095663, 73.48469228349535, 72.80109889280519, 72.11102550927978, 71.41428428542851, 70.71067811865476, 70.0, 69.28203230275508, 68.55654600401044, 67.82329983125268, 67.0820393249937, 66.33249580710799, 65.57438524302, 64.8074069840786, 64.03124237432849, 63.24555320336759, 62.44997998398398, 61.64414002968976, 60.82762530298219, 60.0, 59.16079783099616, 58.309518948453004, 57.445626465380286, 56.568542494923804, 55.67764362830022, 54.772255750516614, 53.85164807134504, 52.91502622129181, 51.96152422706632, 50.99019513592785, 50.0, 48.98979485566356, 47.958315233127195, 46.9041575982343, 45.8257569495584, 44.721359549995796, 43.58898943540674, 42.426406871192846, 41.23105625617661, 40.0, 38.72983346207417, 37.416573867739416, 36.05551275463989, 34.64101615137754, 33.166247903553995, 31.622776601683796, 30.0, 28.284271247461902, 26.457513110645905, 24.49489742783178, 22.360679774997898, 20.0, 17.32050807568877, 14.142135623730951, 10.0]

plt.rc("font", size=18)
plt.rc("axes", titlesize=30)
plt.rc("axes", labelsize=22)
plt.rc("xtick", labelsize=24)
plt.rc("ytick", labelsize=24)
plt.rc("legend", fontsize=22)
plt.rc("figure", titlesize=24)

fig1 = plt.figure(figsize=(30,18),dpi=80)
plt.title('Terrain \n')
plt.plot(x,height,'r')
plt.grid(True)
plt.xlabel(r'position [$m$]')
plt.ylabel(r'height [$m$]')
fig1.savefig('terrain.png',orientation='portrait')