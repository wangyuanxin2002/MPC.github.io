function B0 = genB_7dof_0320(dXb,dYb,dYaw,domegaFr,domegaFl,domegaRr,domegaRl,Xb,Yb,Yaw,Tw1,Tw2,Tw3,Tw4,steer,ddXbc,ddYbc)
%genB_7dof_0320
%    B0 = genB_7dof_0320(dXb,dYb,dYaw,domegaFr,domegaFl,domegaRr,domegaRl,Xb,Yb,Yaw,Tw1,Tw2,Tw3,Tw4,STEER,ddXbc,ddYbc)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    25-Mar-2024 16:18:43

t2 = cos(steer);
t3 = sin(steer);
t4 = dXb.^2;
t5 = -dYb;
t6 = steer.*1.0e+1;
t7 = steer.*(3.0./1.0e+1);
t8 = domegaFl.*(1.1e+1./4.0e+1);
t9 = domegaFr.*(1.1e+1./4.0e+1);
t12 = domegaRl.*(5.7e+1./2.0e+2);
t13 = domegaRr.*(5.7e+1./2.0e+2);
t14 = dYaw.*1.133;
t15 = dYaw.*1.597;
t17 = dYaw.*7.68925e-1;
t35 = ddXbc.*2.80098717948718e+1;
t37 = dYaw.*7.94295e-1;
t40 = ddYbc.*3.99539021945118e+1;
t42 = ddXbc.*ddYbc.*1.040282537279875;
t45 = ddXbc.*ddYbc.*1.074605739114632;
t49 = ddYbc.*5.81744191649514e+1;
t10 = -t8;
t11 = -t9;
t16 = dYb+t15;
t18 = -t17;
t19 = dXb+t17;
t23 = t5+t14;
t36 = -t35;
t39 = -t37;
t43 = dXb+t37;
t46 = -t42;
t50 = -t45;
t52 = -t49;
t79 = t4.*4.99090229010989e-2;
t83 = t4.*1.201909770989011e-1;
t137 = ddYbc.*t4.*1.853613802910024e-3;
t138 = ddYbc.*t4.*4.611156906613236e-3;
t20 = dXb+t18;
t21 = t2.*t16;
t22 = t3.*t16;
t25 = t2.*t19;
t26 = t3.*t19;
t27 = 1.0./t19;
t28 = t2.*t23;
t29 = t3.*t23;
t47 = dXb+t39;
t54 = t2.*t43;
t55 = t3.*t43;
t56 = 1.0./t43;
t82 = -t79;
t139 = -t138;
t246 = t36+t40+t46+t79+t137+1.075768974358974e+3;
t250 = t35+t45+t49+t83+t138+1.516331025641026e+3;
t24 = -t21;
t30 = t2.*t20;
t31 = t3.*t20;
t32 = 1.0./t20;
t33 = -t25;
t38 = t23.*t27;
t51 = t26+t28;
t59 = t2.*t47;
t60 = t3.*t47;
t61 = 1.0./t47;
t66 = -1.0./(t25-t29);
t67 = 1.0./(t25-t29).^2;
t87 = t16.*t56;
t93 = t22+t54;
t248 = t35+t40+t46+t82+t137-1.075768974358974e+3;
t253 = t35+t50+t52+t83+t139+1.516331025641026e+3;
t34 = -t30;
t41 = t23.*t32;
t44 = atan(t38);
t53 = t28+t31;
t58 = t29+t33;
t89 = atan(t87);
t90 = t16.*t61;
t95 = t22+t59;
t99 = t24+t55;
t100 = 1.0./t93;
t105 = t24+t60;
t113 = t11+t93;
t133 = (t51.*-1.0e+1)./(t25-t29);
t135 = (t51.*(-3.0./1.0e+1))./(t25-t29);
t48 = atan(t41);
t57 = t44.*1.0e+1;
t63 = t29+t34;
t64 = t44.*(3.0./1.0e+1);
t76 = t13+t58;
t92 = atan(t90);
t94 = t89.*1.0e+1;
t101 = t100.^2;
t103 = t89.*(3.0./1.0e+1);
t106 = 1.0./t95;
t117 = t10+t95;
t120 = t113.^2;
t168 = t100.*(t21-t55).*-1.0e+1;
t170 = t100.*(t21-t55).*(-3.0./1.0e+1);
t172 = t100.*t113.*1.0e+1;
t175 = t100.*t113.*(3.0./1.0e+1);
t62 = t48.*1.0e+1;
t65 = t48.*(3.0./1.0e+1);
t68 = t6+t57;
t69 = 1.0./t63;
t78 = t12+t63;
t81 = t76.^2;
t96 = t92.*1.0e+1;
t97 = -t94;
t107 = t106.^2;
t108 = t92.*(3.0./1.0e+1);
t109 = -t103;
t123 = t117.^2;
t140 = (t76.*-1.0e+1)./(t25-t29);
t143 = (t76.*(-3.0./1.0e+1))./(t25-t29);
t169 = t106.*(t21-t60).*-1.0e+1;
t171 = t106.*(t21-t60).*(-3.0./1.0e+1);
t173 = atan(t172);
t174 = t106.*t117.*1.0e+1;
t176 = t101.*t120.*1.0e+2;
t178 = t106.*t117.*(3.0./1.0e+1);
t181 = t51.*t67.*t76.*1.0e+1;
t185 = t51.*t67.*t76.*(3.0./1.0e+1);
t210 = t101.*t113.*(t21-t55).*-1.0e+1;
t211 = t101.*t113.*(t21-t55).*1.0e+1;
t212 = t101.*t113.*(t21-t55).*(-3.0./1.0e+1);
t214 = t101.*t113.*(t21-t55).*(3.0./1.0e+1);
t70 = t69.^2;
t71 = t6+t62;
t72 = atan(t68);
t73 = t68.^2;
t85 = t78.^2;
t102 = -t96;
t110 = -t108;
t111 = t6+t97;
t134 = t53.*t69.*1.0e+1;
t136 = t53.*t69.*(3.0./1.0e+1);
t141 = atan(t140);
t142 = t69.*t78.*1.0e+1;
t144 = t67.*t81.*1.0e+2;
t146 = t69.*t78.*(3.0./1.0e+1);
t177 = atan(t174);
t179 = t107.*t123.*1.0e+2;
t180 = t176+1.0;
t182 = -t181;
t187 = -t185;
t188 = t173.*(9.7e+1./1.0e+2);
t213 = t107.*t117.*(t21-t60).*-1.0e+1;
t215 = t107.*t117.*(t21-t60).*1.0e+1;
t216 = t107.*t117.*(t21-t60).*(-3.0./1.0e+1);
t217 = t107.*t117.*(t21-t60).*(3.0./1.0e+1);
t254 = t168+t211;
t74 = atan(t71);
t75 = t71.^2;
t77 = t73+1.0;
t88 = t72.*(9.7e+1./1.0e+2);
t114 = t6+t102;
t115 = atan(t111);
t118 = t111.^2;
t145 = atan(t142);
t147 = t70.*t85.*1.0e+2;
t148 = t144+1.0;
t155 = t141.*(9.7e+1./1.0e+2);
t183 = t179+1.0;
t184 = 1.0./t180;
t186 = t53.*t70.*t78.*1.0e+1;
t191 = t53.*t70.*t78.*(3.0./1.0e+1);
t192 = t177.*(9.7e+1./1.0e+2);
t232 = t133+t182;
t234 = t175+t188;
t255 = t169+t215;
t80 = t75+1.0;
t84 = 1.0./t77;
t91 = t74.*(9.7e+1./1.0e+2);
t119 = atan(t114);
t121 = t114.^2;
t122 = t118+1.0;
t127 = t115.*(9.7e+1./1.0e+2);
t149 = t7+t64+t88;
t150 = t147+1.0;
t151 = 1.0./t148;
t160 = t145.*(9.7e+1./1.0e+2);
t189 = 1.0./t183;
t190 = -t186;
t193 = -t191;
t218 = t143+t155;
t235 = atan(t234);
t236 = t234.^2;
t237 = t178+t192;
t261 = t184.*(t210+t100.*(t21-t55).*1.0e+1).*(-9.7e+1./1.0e+2);
t86 = 1.0./t80;
t98 = t84.*(9.7e+1./1.0e+1);
t124 = t121+1.0;
t125 = 1.0./t122;
t128 = t119.*(9.7e+1./1.0e+2);
t152 = atan(t149);
t153 = t7+t65+t91;
t154 = t149.^2;
t156 = 1.0./t150;
t194 = t7+t109+t127;
t219 = atan(t218);
t220 = t218.^2;
t222 = t146+t160;
t233 = t134+t190;
t238 = t236+1.0;
t239 = atan(t237);
t240 = t237.^2;
t242 = t235.*(1.9e+1./1.0e+1);
t256 = t151.*(t181+(t51.*1.0e+1)./(t25-t29)).*(-9.7e+1./1.0e+2);
t262 = t189.*(t213+t106.*(t21-t60).*1.0e+1).*(-9.7e+1./1.0e+2);
t271 = t170+t214+t261;
t104 = t86.*(9.7e+1./1.0e+1);
t112 = t98+3.0./1.0e+1;
t126 = 1.0./t124;
t129 = t125.*(9.7e+1./1.0e+1);
t157 = atan(t153);
t158 = t153.^2;
t159 = t154+1.0;
t163 = t152.*(1.9e+1./1.0e+1);
t195 = atan(t194);
t196 = t194.^2;
t197 = t7+t110+t128;
t221 = t220+1.0;
t223 = atan(t222);
t224 = t222.^2;
t225 = t219.*(1.9e+1./1.0e+1);
t241 = 1.0./t238;
t243 = cos(t242);
t244 = sin(t242);
t245 = t240+1.0;
t249 = t239.*(1.9e+1./1.0e+1);
t257 = t156.*t233.*(9.7e+1./1.0e+2);
t265 = t135+t187+t256;
t272 = t171+t217+t262;
t116 = t104+3.0./1.0e+1;
t130 = t126.*(9.7e+1./1.0e+1);
t131 = t129+3.0./1.0e+1;
t161 = t158+1.0;
t162 = 1.0./t159;
t164 = cos(t163);
t166 = t157.*(1.9e+1./1.0e+1);
t198 = atan(t197);
t199 = t197.^2;
t200 = t196+1.0;
t203 = t195.*(1.9e+1./1.0e+1);
t226 = 1.0./t221;
t227 = t224+1.0;
t228 = cos(t225);
t229 = t223.*(1.9e+1./1.0e+1);
t247 = 1.0./t245;
t251 = cos(t249);
t252 = sin(t249);
t263 = t244.*t246;
t267 = t136+t193+t257;
t276 = t241.*t243.*t246.*(t212+t184.*(t210+t100.*(t21-t55).*1.0e+1).*(9.7e+1./1.0e+2)+t100.*(t21-t55).*(3.0./1.0e+1)).*(-1.9e+1./1.0e+1);
t277 = t241.*t243.*t246.*(t212+t184.*(t210+t100.*(t21-t55).*1.0e+1).*(9.7e+1./1.0e+2)+t100.*(t21-t55).*(3.0./1.0e+1)).*(1.9e+1./1.0e+1);
t132 = t130+3.0./1.0e+1;
t165 = 1.0./t161;
t167 = cos(t166);
t201 = t199+1.0;
t202 = 1.0./t200;
t204 = cos(t203);
t205 = sin(t203);
t207 = t198.*(1.9e+1./1.0e+1);
t230 = 1.0./t227;
t231 = cos(t229);
t264 = -t263;
t266 = t252.*(t35+t40-t42+t82+t137-1.075768974358974e+3);
t278 = t247.*t251.*(t216+t189.*(t213+t106.*(t21-t60).*1.0e+1).*(9.7e+1./1.0e+2)+t106.*(t21-t60).*(3.0./1.0e+1)).*(t35+t40-t42+t82+t137-1.075768974358974e+3).*(-1.9e+1./1.0e+1);
t206 = 1.0./t201;
t208 = cos(t207);
t209 = sin(t207);
t258 = t205.*t246;
t268 = t131.*t202.*t204.*t246.*(1.9e+1./1.0e+1);
t274 = t264+t266;
t279 = t277+t278;
t259 = -t258;
t260 = t209.*(t35+t40-t42+t82+t137-1.075768974358974e+3);
t269 = -t268;
t270 = t132.*t206.*t208.*(t35+t40-t42+t82+t137-1.075768974358974e+3).*(1.9e+1./1.0e+1);
t273 = t259+t260;
t275 = t269+t270;
et1 = t2.*(t258-t260).*(-1.0./5.29e+2)+(t3.*(t263-t266))./5.29e+2-(t3.*(t268-t270))./5.29e+2+(t2.*(t276+t247.*t251.*(t216+t189.*(t213+t106.*(t21-t60).*1.0e+1).*(9.7e+1./1.0e+2)+t106.*(t21-t60).*(3.0./1.0e+1)).*(t35+t40-t42+t82+t137-1.075768974358974e+3).*(1.9e+1./1.0e+1)))./5.29e+2+t230.*t231.*t267.*(t36+t45+t49-t83+t138-1.516331025641026e+3).*3.591682419659735e-3;
et2 = t226.*t228.*t250.*(t185+t151.*(t181+(t51.*1.0e+1)./(t25-t29)).*(9.7e+1./1.0e+2)+(t51.*(3.0./1.0e+1))./(t25-t29)).*3.591682419659735e-3;
et3 = t3.*(t258-t260).*(-1.0./5.29e+2)-(t2.*(t263-t266))./5.29e+2+(t2.*(t268-t270))./5.29e+2+(t3.*(t276+t247.*t251.*(t216+t189.*(t213+t106.*(t21-t60).*1.0e+1).*(9.7e+1./1.0e+2)+t106.*(t21-t60).*(3.0./1.0e+1)).*(t35+t40-t42+t82+t137-1.075768974358974e+3).*(1.9e+1./1.0e+1)))./5.29e+2+t112.*t162.*t164.*t250.*3.591682419659735e-3;
et4 = t116.*t165.*t167.*(t36+t45+t49-t83+t138-1.516331025641026e+3).*(-3.591682419659735e-3);
et5 = t2.*(t247.*t251.*(t216+t189.*(t213+t106.*(t21-t60).*1.0e+1).*(9.7e+1./1.0e+2)+t106.*(t21-t60).*(3.0./1.0e+1)).*(t35+t40-t42+t82+t137-1.075768974358974e+3).*1.5091605+t241.*t243.*t246.*(t212+t184.*(t210+t100.*(t21-t55).*1.0e+1).*(9.7e+1./1.0e+2)+t100.*(t21-t55).*(3.0./1.0e+1)).*1.5091605).*(-1.405876564037677e-3);
et6 = t3.*(t132.*t206.*t208.*(t35+t40-t42+t82+t137-1.075768974358974e+3).*1.5091605+t131.*t202.*t204.*t246.*1.5091605).*(-1.405876564037677e-3)-t3.*t258.*2.245184872768171e-3+t3.*t260.*2.245184872768171e-3-t2.*t263.*2.245184872768171e-3+t2.*t266.*2.245184872768171e-3;
et7 = t2.*(t258.*7.94295e-1+t260.*7.94295e-1).*(-1.405876564037677e-3)+t3.*(t263.*7.94295e-1+t266.*7.94295e-1).*1.405876564037677e-3-t112.*t162.*t164.*t250.*3.026430479403908e-3;
et8 = t116.*t165.*t167.*(t36+t45+t49-t83+t138-1.516331025641026e+3).*3.026430479403908e-3-t230.*t231.*t267.*(t36+t45+t49-t83+t138-1.516331025641026e+3).*2.053925910305075e-3+t226.*t228.*t250.*(t185+t151.*(t181+(t51.*1.0e+1)./(t25-t29)).*(9.7e+1./1.0e+2)+(t51.*(3.0./1.0e+1))./(t25-t29)).*2.053925910305075e-3;
et9 = t3.*t247.*t251.*(t216+t189.*(t213+t106.*(t21-t60).*1.0e+1).*(9.7e+1./1.0e+2)+t106.*(t21-t60).*(3.0./1.0e+1)).*(t35+t40-t42+t82+t137-1.075768974358974e+3).*4.265851258259525e-3-t3.*t241.*t243.*t246.*(t212+t184.*(t210+t100.*(t21-t55).*1.0e+1).*(9.7e+1./1.0e+2)+t100.*(t21-t55).*(3.0./1.0e+1)).*4.265851258259525e-3;
et10 = t2.*t132.*t206.*t208.*(t35+t40-t42+t82+t137-1.075768974358974e+3).*(-4.265851258259525e-3)+t2.*t131.*t202.*t204.*t246.*4.265851258259525e-3;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,2.0,0.0,et1+et2,0.0,et3+et4,0.0,et5+et6+et7+et8+et9+et10,t241.*t243.*t246.*(t212+t184.*(t210+t100.*(t21-t55).*1.0e+1).*(9.7e+1./1.0e+2)+t100.*(t21-t55).*(3.0./1.0e+1)).*(2.09e+2./2.0e+2),t247.*t251.*(t216+t189.*(t213+t106.*(t21-t60).*1.0e+1).*(9.7e+1./1.0e+2)+t106.*(t21-t60).*(3.0./1.0e+1)).*(t35+t40-t42+t82+t137-1.075768974358974e+3).*(-2.09e+2./2.0e+2)];
mt2 = [t226.*t228.*t250.*(t185+t151.*(t181+(t51.*1.0e+1)./(t25-t29)).*(9.7e+1./1.0e+2)+(t51.*(3.0./1.0e+1))./(t25-t29)).*(-1.083),t230.*t231.*t267.*(t36+t45+t49-t83+t138-1.516331025641026e+3).*(-1.083)];
B0 = reshape([mt1,mt2],10,5);