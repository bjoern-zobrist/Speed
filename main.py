#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 15:53:20 2021

@author: bjornzobrist
"""

#if you go slow you die

import functions as fc
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import time
import simpleaudio as sa

start_time = time.time()


a_pmax = 8.0 #maximal parallel acceleration
a_pmin = 12.0 #maximal deceleration
a_smax = 12.0 #maximal orthogonal acceleration
v_max = 20 #maximal speed
method = 0 #0 slsqp, 1 trust-constr, 2 COBYLA
path = fc.track(375,385)

res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method)

print('Calculations done')

Datei = '455602__inspectorj__tripod-horn-blast-single-01.wav'
wave = sa.WaveObject.from_wave_file(Datei)
play = wave.play()
play.wait_done()

alpha, vel = fc.split(res.x)

#alpha = [0.5407832316981136, 0.5321534890355436, 0.5192100242316748, 0.47328992141764986, 0.46411494939136266, 0.48299165155479434, 0.5230171246289688, 0.6079976894133363, 0.6731341222790395, 0.732497044958771, 0.7928761597305721, 0.859073778465479, 0.92439454944601, 0.964468593829135, 0.9972678393751058, 0.9931384056441867, 0.9552269696903857, 0.8927466158509638, 0.8287130670480578, 0.7430873274254786, 0.6102183786988914, 0.43229376529800617, 0.2844862928848067, 0.13114467946004313, 0.049267072983787595, 0.03184031649825473, 0.02024712192608977, 0.036527556124441, 0.0650734653304457, 0.08881015559199977, 0.15753771805274724, 0.2277187372292399, 0.29092778260976454, 0.4184593129474329, 0.4984713863230239, 0.5956014938842649, 0.6502749280243255, 0.7431992016435779, 0.7747885695939652, 0.7866681739914647, 0.8410029384840121, 0.930571709247623, 0.9426895336189208, 0.9234733057021018, 0.9932841380504729, 0.9918672089389983, 0.9417389452951543, 0.8907108038835171, 0.8445482992521266, 0.8103217965176054, 0.7800784546826752, 0.7621474398419963, 0.7516629642368553, 0.7288130447644391, 0.6992151165975268, 0.670381170693688, 0.6494092143071066, 0.6189772117084998, 0.5928908876441855, 0.5682935628264114, 0.5422513653979754, 0.509732644094663, 0.48887393189198086, 0.4841273082434585, 0.4555940202749518, 0.44924451282884903, 0.4624208665223642, 0.4727482175898073, 0.4674789261709439, 0.47606714664925354, 0.4618714159323781, 0.47507944971591803, 0.4643011518406471, 0.4307157687443445, 0.3871182279947982, 0.37156977909292715, 0.3596124586943879, 0.36411915205162015, 0.36369472436711847, 0.3865876140401766, 0.40256370380456896, 0.41066013388910333, 0.4029191577036011, 0.38716819227156646, 0.3836534736035382, 0.38441326826692807, 0.39174253857774916, 0.40163621014383855, 0.41121995522105104, 0.4162754988144187, 0.4205742796626123, 0.4213598259345787, 0.4150420974154061, 0.407322770464785, 0.3881515082222776, 0.367780015619226, 0.3462477284768127, 0.32834888303303744, 0.3084546984966948, 0.28636253432094033, 0.2620452595008275, 0.23675485525134907, 0.2103509230318101, 0.19841316536875683, 0.21061131533048577, 0.23163752506334495, 0.2515526857701837, 0.26276981492074764, 0.2778420740904, 0.30093603187834295, 0.3439653644202932, 0.4008497615184348, 0.49618477653864324, 0.5872380444442733, 0.654660174719995, 0.7137760921454707, 0.7595517237375531, 0.7839672972680644, 0.7824723457401741, 0.7497531011232202, 0.6792750743369695, 0.5797564278554822, 0.4574979636364647, 0.32222463157320247, 0.2045300187666657, 0.09324661298585951, 0.0175191777534139, 0.024289523080249252, 0.005595916311962252, 0.015790752623436738, 0.019037113874458018, 0.0443790885263983, 0.02240577766711113, 0.03191995495543176, 0.027340942011020605, 0.034917597614932246, 0.03306475684855892, 0.016241538644165884, 0.045177080495960704, 0.062125416731262026, 0.1514190282259743, 0.3022589533633588, 0.4054698653479843, 0.5271492054120399, 0.599253627649027, 0.6614142121525404, 0.7478724047014443, 0.8175820990226605, 0.8491053961432199, 0.9032230613847353, 0.9466325558834358, 0.9597465948647887, 0.954498728893866, 0.9433184113019503, 0.9298117482794752, 0.9372398449013915, 0.9123779649968491, 0.8722570987288162, 0.8306431560063692, 0.7579085682377207, 0.6812828732216539, 0.6021684221318825, 0.5464118811977545, 0.4745797303453243, 0.40088690195418586, 0.3186202788250747, 0.25817266666566685, 0.19075417951915108, 0.13433589988157899, 0.10613954095652196, 0.1007410427949865, 0.12641956526716475, 0.18982318575703896, 0.2978041799573256, 0.4632602664600604, 0.7054936252042375, 0.9722955638869581, 0.9996241806934448, 0.925534444813633, 0.8484274664603731, 0.6236856779887375, 0.4229282714678809, 0.2505952428446868, 0.1111110846788502, 0.010417697628431151, 0.0014461013936267208, 0.028022337303040688, 0.125662024298483, 0.22578747134295418, 0.2854239801916461, 0.2887260432554112, 0.25165052434989543, 0.19811473128945042, 0.1607702469498408, 0.11013183340364431, 0.07736761358526976, 0.04395358766488477, 0.04328537083797669, 0.04232911687817344, 0.05315819681146631, 0.11467064669313368]
#vel = [12.541017733137112, 12.691759428926, 12.944487609271532, 13.109124954651987, 13.368384257573856, 12.990312199587796, 12.892191482275207, 12.464506048062356, 12.134521851149751, 12.132024851842868, 11.62411986377554, 11.228580716953807, 11.16585286527483, 11.37982904925726, 11.625389213879425, 12.008909612658597, 12.27217618023021, 12.709372068286104, 12.696576775463091, 12.568812394043823, 12.503957567707625, 11.949441448252747, 11.506542223723809, 10.877949519758667, 10.342435281548381, 9.88214482276617, 9.515535284059075, 9.14338351790175, 9.734644111169954, 10.147851823971191, 10.89809258863292, 11.457883828277495, 11.736566438316034, 11.796672172610547, 12.381280578571182, 12.52892604790977, 12.010287377392602, 12.45082370923579, 12.758661091921656, 12.166327122518027, 11.718126023637867, 11.525784006744654, 11.558225960051606, 11.384847017520551, 11.818683985051198, 12.320502779472376, 12.73498941520504, 13.108509851557415, 13.525020350790545, 13.574517202247016, 13.780654739282499, 13.542812814774297, 13.840089547662952, 13.680188063777162, 13.712186502803709, 13.2358662849012, 13.490353003656075, 13.074291898941242, 13.163274388531223, 12.810382971194942, 12.606068337007486, 11.890179664296284, 11.145003179932166, 10.265114346022262, 9.944308310662377, 9.960304309548818, 9.92671899210733, 9.668208316267306, 9.592543655742741, 10.063306351449624, 10.5205383713205, 10.697133362559375, 10.898592209453156, 11.312379689767075, 11.180756483091905, 10.77340342174096, 11.142198061430747, 11.623909128688101, 12.137930811570358, 12.640611245849577, 13.068773634492274, 13.420092132721756, 13.805684749074851, 14.06408880031521, 14.412222758297446, 14.581033878653878, 14.726151987306249, 14.668281069419844, 14.458174883619483, 14.727257984898836, 15.01062823502617, 15.344002936237635, 15.506382073961634, 15.592544353681872, 15.752722922063489, 15.711937160750752, 15.359691743706955, 14.926463503600038, 14.638811041438622, 14.257140577877193, 13.90492953789585, 13.444808608910476, 13.234109943773042, 13.392126445771764, 12.979878482241865, 13.362541783621094, 13.463238551768598, 13.006832807062661, 12.930634539961497, 13.075039054044181, 13.386299767106228, 13.605289563466384, 13.799144444014367, 13.712493252194283, 13.58726277706464, 13.843266424989606, 13.543593442782555, 13.187418413797234, 13.276247799065516, 12.86141396272338, 12.168494649625874, 11.638359431159692, 11.902746107404084, 11.326759444734629, 11.180207071662592, 11.50775977129685, 11.230353684287318, 10.734357663354544, 10.529426310433637, 10.813530341315094, 10.33164018278298, 9.533463807862198, 8.62383106101456, 9.253533987794553, 10.022485162700823, 10.686133792376046, 11.207524459982913, 11.540082233756552, 11.668410937127517, 11.935722753938144, 12.103359796642987, 11.99354147912324, 11.689317325303813, 12.084276474435933, 12.462697070448952, 12.545887371930537, 12.11217225037072, 11.726200190566411, 11.880173419012198, 11.600548221208255, 12.055810875423303, 12.41784958327416, 12.558473241008127, 12.486944196227453, 12.362201468686589, 12.167030586960212, 12.324952288912169, 12.65976977594157, 12.996105192250972, 13.203736275314801, 13.399351603964577, 13.148044853598924, 13.052526305276723, 12.838200986297704, 13.137391720499565, 13.364069135485076, 13.294108388871418, 13.171760863510366, 13.195431724963512, 13.09190665316887, 12.736704756088464, 12.09609103494983, 11.24937740240714, 10.21970919265165, 8.94530012200511, 7.666269519402527, 6.522484098154157, 8.228015045235768, 9.181250832881537, 10.009472647087653, 10.793684842756425, 9.970601448681277, 9.066155944066024, 7.871423869898727, 6.7319258498297225, 8.003309228683253, 9.04547865919716, 9.892748498029594, 10.551644206159104, 11.194307257155854, 11.756393514049954, 12.210098542979287, 12.546729594855206, 12.87735969010141, 12.851602847488099, 13.012183830038872, 13.133714775890207, 12.698714941586962, 16.63283501466202]
position = np.array(fc.pos(path,alpha))
a_s = []
a_p = []
for i in range(len(path[0])-2):
    #take the correct alphas
    q = alpha[i]
    w = alpha[i+1]
    e = alpha[i+2]
    a_s.append(fc.acc(path,[q,w,e],[vel[i],vel[i+1]],i)[0])
    a_p.append(fc.acc(path,[q,w,e],[vel[i],vel[i+1]],i)[1])
    
t = np.sum(np.array(fc.dist(fc.pos(path,alpha)))/np.array(vel))
    
fc.plotter(path,position,vel,t)

fc.check(a_s,a_smax,0)
print(a_s)
fc.check(a_p,a_pmax,a_pmin)
print(a_p)
print(alpha)
print(vel)

print("--- %s seconds ---" % (time.time() - start_time))
