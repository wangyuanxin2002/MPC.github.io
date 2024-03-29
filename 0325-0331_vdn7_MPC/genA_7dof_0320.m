function A0 = genA_7dof_0320(dXb,dYb,dYaw,domegaFr,domegaFl,domegaRr,domegaRl,Xb,Yb,Yaw,Tw1,Tw2,Tw3,Tw4,steer,ddXbc,ddYbc)
%genA_7dof_0320
%    A0 = genA_7dof_0320(dXb,dYb,dYaw,domegaFr,domegaFl,domegaRr,domegaRl,Xb,Yb,Yaw,Tw1,Tw2,Tw3,Tw4,STEER,ddXbc,ddYbc)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    25-Mar-2024 16:18:37

t2 = cos(Yaw);
t3 = sin(Yaw);
t4 = cos(steer);
t5 = sin(steer);
t6 = dXb.^2;
t7 = -dYb;
t8 = steer.*1.0e+1;
t9 = steer.*(3.0./1.0e+1);
t10 = domegaFl.*(1.1e+1./4.0e+1);
t11 = domegaFr.*(1.1e+1./4.0e+1);
t14 = domegaRl.*(5.7e+1./2.0e+2);
t15 = domegaRr.*(5.7e+1./2.0e+2);
t16 = dYaw.*1.133;
t17 = dYaw.*1.597;
t23 = dYaw.*7.68925e-1;
t46 = ddXbc.*2.80098717948718e+1;
t48 = dYaw.*7.94295e-1;
t52 = ddYbc.*3.99539021945118e+1;
t54 = ddXbc.*ddYbc.*1.040282537279875;
t57 = ddXbc.*ddYbc.*1.074605739114632;
t62 = ddYbc.*5.81744191649514e+1;
t89 = dXb.*9.98180458021978e-2;
t94 = dXb.*2.403819541978022e-1;
t186 = dXb.*ddYbc.*3.707227605820048e-3;
t188 = dXb.*ddYbc.*9.222313813226472e-3;
t12 = -t10;
t13 = -t11;
t18 = t5.*1.133;
t19 = t5.*1.597;
t22 = dYb+t17;
t24 = t4.*7.68925e-1;
t25 = -t23;
t26 = dXb+t23;
t30 = t7+t16;
t47 = -t46;
t49 = t4.*7.94295e-1;
t51 = -t48;
t55 = dXb+t48;
t58 = -t54;
t63 = -t57;
t65 = -t62;
t103 = t6.*4.99090229010989e-2;
t112 = t6.*1.201909770989011e-1;
t187 = -t186;
t189 = -t188;
t190 = ddYbc.*t6.*1.853613802910024e-3;
t191 = ddYbc.*t6.*4.611156906613236e-3;
t286 = t89+t186;
t290 = t94+t188;
t20 = -t18;
t21 = -t19;
t27 = t22.^2;
t28 = dXb+t25;
t29 = t5.*t22;
t31 = t30.^2;
t32 = t4.*t26;
t33 = 1.0./t26;
t35 = t5.*t30;
t41 = t18+t24;
t59 = dXb+t51;
t66 = t4.*t55;
t67 = 1.0./t55;
t97 = t19+t49;
t111 = -t103;
t192 = -t191;
t288 = t89+t187;
t293 = t94+t189;
t384 = t47+t52+t58+t103+t190+1.075768974358974e+3;
t388 = t46+t57+t62+t112+t191+1.516331025641026e+3;
t34 = t33.^2;
t36 = t4.*t28;
t37 = 1.0./t28;
t39 = -t32;
t42 = t33.*1.133;
t43 = t20+t24;
t50 = t30.*t33;
t68 = t67.^2;
t72 = t4.*t59;
t73 = 1.0./t59;
t82 = -1.0./(t32-t35);
t83 = 1.0./(t32-t35).^2;
t99 = (t4.*-1.0e+1)./(t32-t35);
t100 = (t5.*-1.0e+1)./(t32-t35);
t102 = t67.*1.597;
t104 = t21+t49;
t113 = (t4.*(-3.0./1.0e+1))./(t32-t35);
t114 = (t5.*(-3.0./1.0e+1))./(t32-t35);
t119 = (-8.55e-2)./(t32-t35);
t122 = t22.*t67;
t130 = t29+t66;
t386 = t46+t52+t58+t111+t190-1.075768974358974e+3;
t392 = t46+t63+t65+t112+t192+1.516331025641026e+3;
t38 = t37.^2;
t40 = -t36;
t44 = t37.*1.133;
t45 = -t42;
t53 = t30.*t37;
t56 = atan(t50);
t60 = t31.*t34;
t71 = t35+t39;
t74 = t73.^2;
t90 = t30.*t34.*7.68925e-1;
t108 = t73.*1.597;
t109 = -t102;
t124 = atan(t122);
t125 = t22.*t73;
t127 = t27.*t68;
t133 = t29+t72;
t137 = 1.0./t130;
t151 = t13+t130;
t177 = (t43.*-1.0e+1)./(t32-t35);
t180 = (t43.*(-3.0./1.0e+1))./(t32-t35);
t184 = t22.*t68.*7.94295e-1;
t61 = atan(t53);
t64 = t31.*t38;
t69 = t60+1.0;
t70 = t56.*1.0e+1;
t77 = t35+t40;
t78 = t56.*(3.0./1.0e+1);
t93 = t30.*t38.*7.68925e-1;
t96 = t15+t71;
t128 = atan(t125);
t129 = t27.*t74;
t131 = t127+1.0;
t132 = t124.*1.0e+1;
t138 = t137.^2;
t140 = t124.*(3.0./1.0e+1);
t141 = 1.0./t133;
t149 = t4.*t137.*1.0e+1;
t150 = t5.*t137.*1.0e+1;
t156 = t12+t133;
t160 = t4.*t137.*(3.0./1.0e+1);
t161 = t5.*t137.*(3.0./1.0e+1);
t162 = t137.*(3.3e+1./4.0e+2);
t163 = t151.^2;
t178 = t45+t90;
t185 = t22.*t74.*7.94295e-1;
t238 = t97.*t137.*1.0e+1;
t243 = t97.*t137.*(3.0./1.0e+1);
t258 = t137.*t151.*1.0e+1;
t262 = t137.*t151.*(3.0./1.0e+1);
t292 = t109+t184;
t75 = t64+1.0;
t76 = t61.*1.0e+1;
t79 = 1.0./t69;
t80 = t61.*(3.0./1.0e+1);
t84 = t8+t70;
t85 = 1.0./t77;
t101 = t14+t77;
t110 = t96.^2;
t134 = t129+1.0;
t135 = t128.*1.0e+1;
t136 = -t132;
t142 = t141.^2;
t143 = t128.*(3.0./1.0e+1);
t144 = -t140;
t145 = 1.0./t131;
t154 = t4.*t141.*1.0e+1;
t155 = t5.*t141.*1.0e+1;
t166 = t4.*t141.*(3.0./1.0e+1);
t167 = t5.*t141.*(3.0./1.0e+1);
t168 = t141.*(3.3e+1./4.0e+2);
t170 = t156.^2;
t181 = t44+t93;
t193 = (t96.*-1.0e+1)./(t32-t35);
t196 = (t96.*(-3.0./1.0e+1))./(t32-t35);
t197 = t4.*t83.*t96.*1.0e+1;
t198 = t5.*t83.*t96.*1.0e+1;
t205 = t4.*t83.*t96.*(3.0./1.0e+1);
t207 = t5.*t83.*t96.*(3.0./1.0e+1);
t242 = t104.*t141.*1.0e+1;
t244 = t104.*t141.*(3.0./1.0e+1);
t249 = t43.*t83.*t96.*1.0e+1;
t254 = t43.*t83.*t96.*(3.0./1.0e+1);
t260 = atan(t258);
t261 = t141.*t156.*1.0e+1;
t263 = t4.*t138.*t151.*1.0e+1;
t264 = t5.*t138.*t151.*1.0e+1;
t265 = t138.*t163.*1.0e+2;
t269 = t141.*t156.*(3.0./1.0e+1);
t271 = t4.*t138.*t151.*(3.0./1.0e+1);
t273 = t5.*t138.*t151.*(3.0./1.0e+1);
t296 = t108+t185;
t319 = t97.*t138.*t151.*1.0e+1;
t322 = t97.*t138.*t151.*(3.0./1.0e+1);
t81 = 1.0./t75;
t86 = t85.^2;
t87 = t8+t76;
t88 = atan(t84);
t91 = t84.^2;
t106 = t4.*t85.*1.0e+1;
t107 = t5.*t85.*1.0e+1;
t116 = t101.^2;
t117 = t4.*t85.*(3.0./1.0e+1);
t118 = t5.*t85.*(3.0./1.0e+1);
t121 = t85.*8.55e-2;
t139 = -t135;
t146 = -t143;
t147 = 1.0./t134;
t148 = t8+t136;
t157 = t33.*t79.*(3.0./1.0e+1);
t176 = t41.*t85.*1.0e+1;
t179 = t41.*t85.*(3.0./1.0e+1);
t182 = t30.*t34.*t79.*(3.0./1.0e+1);
t194 = atan(t193);
t195 = t85.*t101.*1.0e+1;
t199 = t83.*t110.*1.0e+2;
t201 = -t197;
t202 = -t198;
t203 = t85.*t101.*(3.0./1.0e+1);
t211 = -t205;
t213 = -t207;
t227 = t67.*t145.*(3.0./1.0e+1);
t245 = t22.*t68.*t145.*(3.0./1.0e+1);
t250 = t79.*(t42-t90).*(-3.0./1.0e+1);
t252 = -t249;
t257 = -t254;
t266 = atan(t261);
t267 = -t263;
t268 = -t264;
t270 = t4.*t142.*t156.*1.0e+1;
t272 = t5.*t142.*t156.*1.0e+1;
t274 = t142.*t170.*1.0e+2;
t275 = t265+1.0;
t277 = -t271;
t279 = -t273;
t280 = t4.*t142.*t156.*(3.0./1.0e+1);
t281 = t5.*t142.*t156.*(3.0./1.0e+1);
t289 = t260.*(9.7e+1./1.0e+2);
t320 = -t319;
t324 = t104.*t142.*t156.*1.0e+1;
t325 = -t322;
t328 = t104.*t142.*t156.*(3.0./1.0e+1);
t342 = t145.*(t102-t184).*(-3.0./1.0e+1);
t92 = atan(t87);
t95 = t87.^2;
t98 = t91+1.0;
t123 = t88.*(9.7e+1./1.0e+2);
t152 = t8+t139;
t153 = atan(t148);
t158 = t148.^2;
t169 = t37.*t81.*(3.0./1.0e+1);
t183 = t30.*t38.*t81.*(3.0./1.0e+1);
t200 = atan(t195);
t204 = t4.*t86.*t101.*1.0e+1;
t206 = t5.*t86.*t101.*1.0e+1;
t208 = t86.*t116.*1.0e+2;
t209 = t199+1.0;
t214 = t4.*t86.*t101.*(3.0./1.0e+1);
t215 = t5.*t86.*t101.*(3.0./1.0e+1);
t224 = t194.*(9.7e+1./1.0e+2);
t231 = t73.*t147.*(3.0./1.0e+1);
t247 = t22.*t74.*t147.*(3.0./1.0e+1);
t251 = t41.*t86.*t101.*1.0e+1;
t255 = t81.*t181.*(3.0./1.0e+1);
t256 = t41.*t86.*t101.*(3.0./1.0e+1);
t276 = -t270;
t278 = -t272;
t282 = t274+1.0;
t283 = -t280;
t284 = -t281;
t285 = 1.0./t275;
t295 = t266.*(9.7e+1./1.0e+2);
t306 = t99+t201;
t307 = t100+t202;
t326 = -t324;
t330 = -t328;
t335 = t149+t267;
t336 = t150+t268;
t339 = t177+t252;
t346 = t147.*t296.*(3.0./1.0e+1);
t369 = t262+t289;
t396 = t238+t320;
t105 = t95+1.0;
t115 = 1.0./t98;
t126 = t92.*(9.7e+1./1.0e+2);
t159 = atan(t152);
t164 = t152.^2;
t165 = t158+1.0;
t174 = t153.*(9.7e+1./1.0e+2);
t210 = -t204;
t212 = -t206;
t216 = t9+t78+t123;
t217 = t208+1.0;
t218 = -t214;
t219 = -t215;
t220 = 1.0./t209;
t230 = t200.*(9.7e+1./1.0e+2);
t253 = -t251;
t259 = -t256;
t291 = 1.0./t282;
t337 = t154+t276;
t338 = t155+t278;
t341 = t196+t224;
t344 = t137.*t285.*2.6675;
t370 = atan(t369);
t371 = t369.^2;
t372 = t269+t295;
t398 = t242+t326;
t405 = t285.*t335.*(9.7e+1./1.0e+2);
t406 = t285.*t336.*(9.7e+1./1.0e+2);
t415 = t285.*t396.*(9.7e+1./1.0e+2);
t120 = 1.0./t105;
t171 = t164+1.0;
t172 = 1.0./t165;
t175 = t159.*(9.7e+1./1.0e+2);
t221 = atan(t216);
t222 = t9+t80+t126;
t223 = t216.^2;
t225 = 1.0./t217;
t246 = t33.*t79.*t115.*(9.7e+1./1.0e+1);
t287 = t30.*t34.*t79.*t115.*(9.7e+1./1.0e+1);
t297 = t9+t144+t174;
t315 = t106+t210;
t316 = t107+t212;
t317 = (t220.*(-2.7645))./(t32-t35);
t323 = t79.*t115.*(t42-t90).*(-9.7e+1./1.0e+1);
t340 = t176+t253;
t343 = atan(t341);
t345 = t341.^2;
t348 = t203+t230;
t349 = t141.*t291.*2.6675;
t365 = t220.*(t197+(t4.*1.0e+1)./(t32-t35)).*(-9.7e+1./1.0e+2);
t366 = t220.*(t198+(t5.*1.0e+1)./(t32-t35)).*(-9.7e+1./1.0e+2);
t373 = t371+1.0;
t374 = t162+t344;
t375 = atan(t372);
t376 = t372.^2;
t378 = t370.*(1.9e+1./1.0e+1);
t402 = t220.*(t249+(t43.*1.0e+1)./(t32-t35)).*(-9.7e+1./1.0e+2);
t407 = t291.*t337.*(9.7e+1./1.0e+2);
t408 = t291.*t338.*(9.7e+1./1.0e+2);
t416 = t291.*t398.*(9.7e+1./1.0e+2);
t421 = t160+t277+t405;
t422 = t161+t279+t406;
t425 = t243+t325+t415;
t173 = 1.0./t171;
t226 = atan(t222);
t228 = t222.^2;
t229 = t223+1.0;
t234 = t221.*(1.9e+1./1.0e+1);
t248 = t37.*t81.*t120.*(9.7e+1./1.0e+1);
t294 = t30.*t38.*t81.*t120.*(9.7e+1./1.0e+1);
t298 = atan(t297);
t299 = t297.^2;
t300 = t9+t146+t175;
t318 = t85.*t225.*2.7645;
t321 = t67.*t145.*t172.*(9.7e+1./1.0e+1);
t329 = t81.*t120.*t181.*(9.7e+1./1.0e+1);
t331 = t157+t246;
t332 = t22.*t68.*t145.*t172.*(9.7e+1./1.0e+1);
t347 = t345+1.0;
t350 = atan(t348);
t351 = t348.^2;
t352 = t343.*(1.9e+1./1.0e+1);
t357 = t119+t317;
t363 = t182+t287;
t367 = t225.*t315.*(9.7e+1./1.0e+2);
t368 = t225.*t316.*(9.7e+1./1.0e+2);
t377 = 1.0./t373;
t379 = cos(t378);
t380 = sin(t378);
t381 = t376+1.0;
t382 = t168+t349;
t383 = t145.*t172.*(t102-t184).*(-9.7e+1./1.0e+1);
t387 = t375.*(1.9e+1./1.0e+1);
t399 = t250+t323;
t403 = t225.*t340.*(9.7e+1./1.0e+2);
t409 = t114+t213+t366;
t410 = t113+t211+t365;
t419 = t180+t257+t402;
t423 = t166+t283+t407;
t424 = t167+t284+t408;
t426 = t244+t330+t416;
t232 = t228+1.0;
t233 = 1.0./t229;
t235 = cos(t234);
t236 = sin(t234);
t239 = t226.*(1.9e+1./1.0e+1);
t301 = atan(t300);
t302 = t300.^2;
t303 = t299+1.0;
t308 = t298.*(1.9e+1./1.0e+1);
t327 = t73.*t147.*t173.*(9.7e+1./1.0e+1);
t333 = t169+t248;
t334 = t22.*t74.*t147.*t173.*(9.7e+1./1.0e+1);
t353 = 1.0./t347;
t354 = t351+1.0;
t355 = cos(t352);
t356 = sin(t352);
t358 = t350.*(1.9e+1./1.0e+1);
t362 = t121+t318;
t364 = t183+t294;
t385 = 1.0./t381;
t389 = cos(t387);
t390 = sin(t387);
t391 = t147.*t173.*t296.*(9.7e+1./1.0e+1);
t393 = t227+t321;
t400 = t255+t329;
t401 = t245+t332;
t411 = t286.*t380;
t413 = t118+t219+t368;
t414 = t117+t218+t367;
t417 = t342+t383;
t420 = t179+t259+t403;
t435 = t377.*t379.*t384.*t421.*(1.9e+1./1.0e+1);
t436 = t377.*t379.*t384.*t422.*(1.9e+1./1.0e+1);
t441 = t377.*t379.*t384.*t425.*(1.9e+1./1.0e+1);
t237 = 1.0./t232;
t240 = cos(t239);
t241 = sin(t239);
t304 = t302+1.0;
t305 = 1.0./t303;
t309 = cos(t308);
t310 = sin(t308);
t312 = t301.*(1.9e+1./1.0e+1);
t359 = 1.0./t354;
t360 = cos(t358);
t361 = sin(t358);
t397 = t231+t327;
t404 = t247+t334;
t412 = t288.*t390;
t418 = t346+t391;
t437 = -t436;
t438 = t385.*t389.*t423.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t439 = t385.*t389.*t424.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t442 = t385.*t389.*t426.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t311 = 1.0./t304;
t313 = cos(t312);
t314 = sin(t312);
t394 = t286.*t310;
t427 = t305.*t309.*t384.*t393.*(1.9e+1./1.0e+1);
t430 = t305.*t309.*t384.*t401.*(1.9e+1./1.0e+1);
t433 = t305.*t309.*t384.*(t145.*(t102-t184).*(3.0./1.0e+1)+t145.*t172.*(t102-t184).*(9.7e+1./1.0e+1)).*(-1.9e+1./1.0e+1);
t440 = -t438;
t446 = t437+t439;
t447 = t441+t442;
t395 = t288.*t314;
t428 = -t427;
t429 = t311.*t313.*t397.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t431 = t311.*t313.*t404.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t434 = t311.*t313.*t418.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t448 = t411+t412+t435+t440;
t432 = -t431;
t443 = t428+t429;
t444 = t433+t434;
t445 = t394+t395+t430+t432;
et1 = t4.*t394.*2.245184872768171e-3+t4.*t395.*2.245184872768171e-3-t5.*t411.*2.245184872768171e-3-t5.*t412.*2.245184872768171e-3-t236.*t290.*1.592858147054689e-3-t241.*t293.*1.592858147054689e-3-t290.*t356.*1.081013637002671e-3+t293.*t361.*1.081013637002671e-3;
et2 = t5.*(t394.*7.94295e-1-t395.*7.94295e-1+t311.*t313.*t404.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*1.5091605+t305.*t309.*t384.*t401.*1.5091605).*(-1.405876564037677e-3);
et3 = t4.*(t411.*7.94295e-1-t412.*7.94295e-1+t385.*t389.*t423.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*1.5091605+t377.*t379.*t384.*t421.*1.5091605).*(-1.405876564037677e-3);
et4 = t233.*t235.*t363.*t388.*3.026430479403908e-3-t237.*t240.*t364.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.026430479403908e-3+t359.*t360.*t414.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*2.053925910305075e-3-t353.*t355.*t388.*(t205+t220.*(t197+(t4.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t4.*(3.0./1.0e+1))./(t32-t35)).*2.053925910305075e-3;
et5 = t4.*t311.*t313.*t404.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(-4.265851258259525e-3)+t5.*t385.*t389.*t423.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*4.265851258259525e-3+t4.*t305.*t309.*t384.*t401.*4.265851258259525e-3-t5.*t377.*t379.*t384.*t421.*4.265851258259525e-3;
et6 = t5.*(t311.*t313.*t397.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*1.5091605+t305.*t309.*t384.*t393.*1.5091605).*1.405876564037677e-3;
et7 = t4.*(t385.*t389.*t424.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*1.5091605+t377.*t379.*t384.*t422.*1.5091605).*(-1.405876564037677e-3)+t233.*t235.*t331.*t388.*3.026430479403908e-3;
et8 = t237.*t240.*t333.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*(-3.026430479403908e-3)+t359.*t360.*t413.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*2.053925910305075e-3-t353.*t355.*t388.*(t207+t220.*(t198+(t5.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t5.*(3.0./1.0e+1))./(t32-t35)).*2.053925910305075e-3;
et9 = t4.*t311.*t313.*t397.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*4.265851258259525e-3+t5.*t385.*t389.*t424.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*4.265851258259525e-3-t4.*t305.*t309.*t384.*t393.*4.265851258259525e-3-t5.*t377.*t379.*t384.*t422.*4.265851258259525e-3;
et10 = t4.*(t385.*t389.*t426.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*1.5091605-t377.*t379.*t384.*t425.*1.5091605).*1.405876564037677e-3;
et11 = t5.*(t311.*t313.*t418.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*1.5091605+t305.*t309.*t384.*(t145.*(t102-t184).*(3.0./1.0e+1)+t145.*t172.*(t102-t184).*(9.7e+1./1.0e+1)).*1.5091605).*1.405876564037677e-3-t233.*t235.*t388.*(t79.*(t42-t90).*(3.0./1.0e+1)+t79.*t115.*(t42-t90).*(9.7e+1./1.0e+1)).*3.026430479403908e-3;
et12 = t237.*t240.*t400.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.026430479403908e-3-t359.*t360.*t420.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*2.053925910305075e-3-t353.*t355.*t388.*(t254+t220.*(t249+(t43.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t43.*(3.0./1.0e+1))./(t32-t35)).*2.053925910305075e-3;
et13 = t4.*t311.*t313.*t418.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*4.265851258259525e-3-t5.*t385.*t389.*t426.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*4.265851258259525e-3-t5.*t377.*t379.*t384.*t425.*4.265851258259525e-3-t4.*t305.*t309.*t384.*(t145.*(t102-t184).*(3.0./1.0e+1)+t145.*t172.*(t102-t184).*(9.7e+1./1.0e+1)).*4.265851258259525e-3;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2,dXb.*(-1.537697875236295e-3)-(t5.*t445)./5.29e+2-(t4.*t448)./5.29e+2-(t290.*t356)./5.29e+2-(t293.*t361)./5.29e+2-t359.*t360.*t414.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3-t353.*t355.*t388.*(t205+t220.*(t197+(t4.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t4.*(3.0./1.0e+1))./(t32-t35)).*3.591682419659735e-3,t3];
mt2 = [(t4.*t445)./5.29e+2-(t5.*t448)./5.29e+2+(t236.*t290)./5.29e+2+(t241.*t293)./5.29e+2-t233.*t235.*t363.*t388.*3.591682419659735e-3+t237.*t240.*t364.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3,0.0,et1+et2+et3+et4+et5,t411.*(1.1e+1./2.0e+1)+t377.*t379.*t384.*t421.*(2.09e+2./2.0e+2),t412.*(1.1e+1./2.0e+1)-t385.*t389.*t423.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(2.09e+2./2.0e+2)];
mt3 = [t290.*t356.*(5.7e+1./1.0e+2)+t353.*t355.*t388.*(t205+t220.*(t197+(t4.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t4.*(3.0./1.0e+1))./(t32-t35)).*1.083,t293.*t361.*(5.7e+1./1.0e+2)+t359.*t360.*t414.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*1.083,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t3];
mt4 = [(t5.*(t427-t429))./5.29e+2-(t4.*(t436-t439))./5.29e+2-t359.*t360.*t413.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3-t353.*t355.*t388.*(t207+t220.*(t198+(t5.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t5.*(3.0./1.0e+1))./(t32-t35)).*3.591682419659735e-3,t2];
mt5 = [t4.*(t427-t429).*(-1.0./5.29e+2)-(t5.*(t436-t439))./5.29e+2-t233.*t235.*t331.*t388.*3.591682419659735e-3+t237.*t240.*t333.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3,0.0,et6+et7+et8+et9,t377.*t379.*t384.*t422.*(2.09e+2./2.0e+2),t385.*t389.*t424.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(-2.09e+2./2.0e+2)];
mt6 = [t353.*t355.*t388.*(t207+t220.*(t198+(t5.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t5.*(3.0./1.0e+1))./(t32-t35)).*1.083,t359.*t360.*t413.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*1.083,-dXb.*t3+t2.*t7,0.0,dXb.*t2+t3.*t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt7 = [t5.*t444.*(-1.0./5.29e+2)-(t4.*t447)./5.29e+2+t359.*t360.*t420.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3-t353.*t355.*t388.*(t254+t220.*(t249+(t43.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t43.*(3.0./1.0e+1))./(t32-t35)).*3.591682419659735e-3,0.0];
mt8 = [(t4.*t444)./5.29e+2-(t5.*t447)./5.29e+2+t233.*t235.*t388.*(t79.*(t42-t90).*(3.0./1.0e+1)+t79.*t115.*(t42-t90).*(9.7e+1./1.0e+1)).*3.591682419659735e-3-t237.*t240.*t400.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3,1.0,et10+et11+et12+et13,t377.*t379.*t384.*t425.*(2.09e+2./2.0e+2),t385.*t389.*t426.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(2.09e+2./2.0e+2)];
mt9 = [t353.*t355.*t388.*(t254+t220.*(t249+(t43.*1.0e+1)./(t32-t35)).*(9.7e+1./1.0e+2)+(t43.*(3.0./1.0e+1))./(t32-t35)).*1.083,t359.*t360.*t420.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*(-1.083),0.0,t4.*t374.*t377.*t379.*t384.*3.591682419659735e-3,0.0,t5.*t374.*t377.*t379.*t384.*3.591682419659735e-3,0.0,t4.*t374.*t377.*t379.*t384.*2.121693378321383e-3+t5.*t374.*t377.*t379.*t384.*4.265851258259525e-3];
mt10 = [t374.*t377.*t379.*t384.*(-2.09e+2./2.0e+2),0.0,0.0,0.0,0.0,t4.*t382.*t385.*t389.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(-3.591682419659735e-3),0.0,t5.*t382.*t385.*t389.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(-3.591682419659735e-3),0.0];
mt11 = [t4.*t382.*t385.*t389.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*2.121693378321383e-3-t5.*t382.*t385.*t389.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*4.265851258259525e-3,0.0,t382.*t385.*t389.*(t46+t52-t54+t111+t190-1.075768974358974e+3).*(2.09e+2./2.0e+2),0.0,0.0,0.0];
mt12 = [t353.*t355.*t388.*(8.55e-2./(t32-t35)+(t220.*2.7645)./(t32-t35)).*3.591682419659735e-3,0.0,0.0,0.0,t353.*t355.*t388.*(8.55e-2./(t32-t35)+(t220.*2.7645)./(t32-t35)).*2.053925910305075e-3,0.0,0.0,t353.*t355.*t388.*(8.55e-2./(t32-t35)+(t220.*2.7645)./(t32-t35)).*(-1.083),0.0,0.0];
mt13 = [t359.*t360.*t362.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*3.591682419659735e-3,0.0,0.0,0.0,t359.*t360.*t362.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*(-2.053925910305075e-3),0.0,0.0,0.0,t359.*t360.*t362.*(t47+t57+t62-t112+t191-1.516331025641026e+3).*(-1.083)];
A0 = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12,mt13],10,10);
