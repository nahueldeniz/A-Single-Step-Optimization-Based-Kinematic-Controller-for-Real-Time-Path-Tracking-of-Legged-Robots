function Status = read_go1_status(go1_obj, Status, bag)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if bag == true
        go1_status_data.Data = go1_obj;
    else
        go1_status_data = receive(go1_obj);
    end
    %
    Status.IMU.quaternion.x = go1_status_data.Data(1);
    Status.IMU.quaternion.y = go1_status_data.Data(2);
    Status.IMU.quaternion.z = go1_status_data.Data(3);
    Status.IMU.quaternion.w = go1_status_data.Data(4);
    %
    Status.IMU.gyro.x = go1_status_data.Data(5);
    Status.IMU.gyro.y = go1_status_data.Data(6);
    Status.IMU.gyro.z = go1_status_data.Data(7);
    %
    Status.IMU.accel.x = go1_status_data.Data(8);
    Status.IMU.accel.y = go1_status_data.Data(9);
    Status.IMU.accel.z = go1_status_data.Data(10);
    %
    Status.IMU.temp = go1_status_data.Data(11);
    %
    Status.IMU.rpy.first = go1_status_data.Data(12);
    Status.IMU.rpy.second = go1_status_data.Data(13);
    Status.IMU.rpy.third = go1_status_data.Data(14);
    %
    Status.BMS.SOC = go1_status_data.Data(15);
    %
    Status.BMS.current = go1_status_data.Data(16);
    %
    Status.BMS.cycle = go1_status_data.Data(17);
    %
    Status.BMS.BQ_NTC.first = go1_status_data.Data(19);
    Status.BMS.BQ_NTC.second = go1_status_data.Data(19);
    %
    Status.BMS.MCS_NTC.first = go1_status_data.Data(20);
    Status.BMS.MCS_NTC.second = go1_status_data.Data(21);
    %
    Status.BMS.vol_cell1 = go1_status_data.Data(22);
    Status.BMS.vol_cell2 = go1_status_data.Data(23);
    Status.BMS.vol_cell3 = go1_status_data.Data(24);
    Status.BMS.vol_cell4 = go1_status_data.Data(25);
    Status.BMS.vol_cell5 = go1_status_data.Data(26);
    Status.BMS.vol_cell6 = go1_status_data.Data(27);
    Status.BMS.vol_cell7 = go1_status_data.Data(28);
    Status.BMS.vol_cell8 = go1_status_data.Data(29);
    Status.BMS.vol_cell9 = go1_status_data.Data(30);
    Status.BMS.vol_cell10 = go1_status_data.Data(31);
    %
    Status.foot_force.foot1 = go1_status_data.Data(32);
    Status.foot_force.foot2 = go1_status_data.Data(33);
    Status.foot_force.foot3 = go1_status_data.Data(34);
    Status.foot_force.foot4 = go1_status_data.Data(35);
    %
    Status.foot_forceEst.foot1 = go1_status_data.Data(36);
    Status.foot_forceEst.foot2 = go1_status_data.Data(37);
    Status.foot_forceEst.foot3 = go1_status_data.Data(38);
    Status.foot_forceEst.foot4 = go1_status_data.Data(39);
    %
    Status.footRaiseHeight = go1_status_data.Data(40);
    %
    Status.position.x = go1_status_data.Data(41);
    Status.position.y = go1_status_data.Data(42);
    Status.position.z = go1_status_data.Data(43);
    %
    Status.bodyHeight = go1_status_data.Data(44);
    %
    Status.velocity.x = go1_status_data.Data(45);
    Status.velocity.y = go1_status_data.Data(46);
    Status.velocity.z = go1_status_data.Data(47);
    %
    Status.yawSpeed = go1_status_data.Data(48);
    %
    Status.rangeObstacle.front = go1_status_data.Data(49);
    Status.rangeObstacle.left = go1_status_data.Data(50);
    Status.rangeObstacle.right = go1_status_data.Data(51);
    Status.rangeObstacle.reserved = go1_status_data.Data(52);
    %
    Status.motorState1.mode = go1_status_data.Data(53);
    Status.motorState1.q = go1_status_data.Data(54);
    Status.motorState1.dq = go1_status_data.Data(55);
    Status.motorState1.ddq = go1_status_data.Data(56);
    Status.motorState1.tauEst = go1_status_data.Data(57);
    Status.motorState1.q_raw = go1_status_data.Data(58);
    Status.motorState1.dq_raw = go1_status_data.Data(59);
    Status.motorState1.ddq_raw = go1_status_data.Data(60);
    %
    Status.motorState2.mode = go1_status_data.Data(61);
    Status.motorState2.q = go1_status_data.Data(62);
    Status.motorState2.dq = go1_status_data.Data(63);
    Status.motorState2.ddq = go1_status_data.Data(64);
    Status.motorState2.tauEst = go1_status_data.Data(65);
    Status.motorState2.q_raw = go1_status_data.Data(66);
    Status.motorState2.dq_raw = go1_status_data.Data(67);
    Status.motorState2.ddq_raw = go1_status_data.Data(68);
    %
    Status.motorState3.mode = go1_status_data.Data(69);
    Status.motorState3.q = go1_status_data.Data(70);
    Status.motorState3.dq = go1_status_data.Data(71);
    Status.motorState3.ddq = go1_status_data.Data(72);
    Status.motorState3.tauEst = go1_status_data.Data(73);
    Status.motorState3.q_raw = go1_status_data.Data(74);
    Status.motorState3.dq_raw = go1_status_data.Data(75);
    Status.motorState3.ddq_raw = go1_status_data.Data(76);
    %
    Status.motorState4.mode = go1_status_data.Data(77);
    Status.motorState4.q = go1_status_data.Data(78);
    Status.motorState4.dq = go1_status_data.Data(79);
    Status.motorState4.ddq = go1_status_data.Data(80);
    Status.motorState4.tauEst = go1_status_data.Data(81);
    Status.motorState4.q_raw = go1_status_data.Data(82);
    Status.motorState4.dq_raw = go1_status_data.Data(83);
    Status.motorState4.ddq_raw = go1_status_data.Data(84);
    %
    Status.motorState5.mode = go1_status_data.Data(85);
    Status.motorState5.q = go1_status_data.Data(86);
    Status.motorState5.dq = go1_status_data.Data(87);
    Status.motorState5.ddq = go1_status_data.Data(88);
    Status.motorState5.tauEst = go1_status_data.Data(89);
    Status.motorState5.q_raw = go1_status_data.Data(90);
    Status.motorState5.dq_raw = go1_status_data.Data(91);
    Status.motorState5.ddq_raw = go1_status_data.Data(92);
    %
    Status.motorState6.mode = go1_status_data.Data(93);
    Status.motorState6.q = go1_status_data.Data(94);
    Status.motorState6.dq = go1_status_data.Data(95);
    Status.motorState6.ddq = go1_status_data.Data(96);
    Status.motorState6.tauEst = go1_status_data.Data(97);
    Status.motorState6.q_raw = go1_status_data.Data(98);
    Status.motorState6.dq_raw = go1_status_data.Data(99);
    Status.motorState6.ddq_raw = go1_status_data.Data(100);
    %
    Status.motorState7.mode = go1_status_data.Data(101);
    Status.motorState7.q = go1_status_data.Data(102);
    Status.motorState7.dq = go1_status_data.Data(103);
    Status.motorState7.ddq = go1_status_data.Data(104);
    Status.motorState7.tauEst = go1_status_data.Data(105);
    Status.motorState7.q_raw = go1_status_data.Data(106);
    Status.motorState7.dq_raw = go1_status_data.Data(107);
    Status.motorState7.ddq_raw = go1_status_data.Data(108);
    %
    Status.motorState8.mode = go1_status_data.Data(109);
    Status.motorState8.q = go1_status_data.Data(110);
    Status.motorState8.dq = go1_status_data.Data(111);
    Status.motorState8.ddq = go1_status_data.Data(112);
    Status.motorState8.tauEst = go1_status_data.Data(113);
    Status.motorState8.q_raw = go1_status_data.Data(114);
    Status.motorState8.dq_raw = go1_status_data.Data(115);
    Status.motorState8.ddq_raw = go1_status_data.Data(116);
    %
    Status.motorState9.mode = go1_status_data.Data(117);
    Status.motorState9.q = go1_status_data.Data(118);
    Status.motorState9.dq = go1_status_data.Data(119);
    Status.motorState9.ddq = go1_status_data.Data(120);
    Status.motorState9.tauEst = go1_status_data.Data(121);
    Status.motorState9.q_raw = go1_status_data.Data(122);
    Status.motorState9.dq_raw = go1_status_data.Data(123);
    Status.motorState9.ddq_raw = go1_status_data.Data(124);
    %
    Status.motorState10.mode = go1_status_data.Data(125);
    Status.motorState10.q = go1_status_data.Data(126);
    Status.motorState10.dq = go1_status_data.Data(127);
    Status.motorState10.ddq = go1_status_data.Data(128);
    Status.motorState10.tauEst = go1_status_data.Data(129);
    Status.motorState10.q_raw = go1_status_data.Data(130);
    Status.motorState10.dq_raw = go1_status_data.Data(131);
    Status.motorState10.ddq_raw = go1_status_data.Data(132);
    %
    Status.motorState11.mode = go1_status_data.Data(133);
    Status.motorState11.q = go1_status_data.Data(134);
    Status.motorState11.dq = go1_status_data.Data(135);
    Status.motorState11.ddq = go1_status_data.Data(136);
    Status.motorState11.tauEst = go1_status_data.Data(137);
    Status.motorState11.q_raw = go1_status_data.Data(138);
    Status.motorState11.dq_raw = go1_status_data.Data(139);
    Status.motorState11.ddq_raw = go1_status_data.Data(140);
    %
    Status.motorState12.mode = go1_status_data.Data(141);
    Status.motorState12.q = go1_status_data.Data(142);
    Status.motorState12.dq = go1_status_data.Data(143);
    Status.motorState12.ddq = go1_status_data.Data(144);
    Status.motorState12.tauEst = go1_status_data.Data(145);
    Status.motorState12.q_raw = go1_status_data.Data(146);
    Status.motorState12.dq_raw = go1_status_data.Data(147);
    Status.motorState12.ddq_raw = go1_status_data.Data(148);
    %
    Status.motorState13.mode = go1_status_data.Data(149);
    Status.motorState13.q = go1_status_data.Data(150);
    Status.motorState13.dq = go1_status_data.Data(151);
    Status.motorState13.ddq = go1_status_data.Data(152);
    Status.motorState13.tauEst = go1_status_data.Data(153);
    Status.motorState13.q_raw = go1_status_data.Data(154);
    Status.motorState13.dq_raw = go1_status_data.Data(155);
    Status.motorState13.ddq_raw = go1_status_data.Data(156);
    %
    Status.motorState14.mode = go1_status_data.Data(158);
    Status.motorState14.q = go1_status_data.Data(158);
    Status.motorState14.dq = go1_status_data.Data(159);
    Status.motorState14.ddq = go1_status_data.Data(160);
    Status.motorState14.tauEst = go1_status_data.Data(161);
    Status.motorState14.q_raw = go1_status_data.Data(162);
    Status.motorState14.dq_raw = go1_status_data.Data(163);
    Status.motorState14.ddq_raw = go1_status_data.Data(164);
    %
    Status.motorState15.mode = go1_status_data.Data(165);
    Status.motorState15.q = go1_status_data.Data(166);
    Status.motorState15.dq = go1_status_data.Data(167);
    Status.motorState15.ddq = go1_status_data.Data(168);
    Status.motorState15.tauEst = go1_status_data.Data(169);
    Status.motorState15.q_raw = go1_status_data.Data(170);
    Status.motorState15.dq_raw = go1_status_data.Data(171);
    Status.motorState15.ddq_raw = go1_status_data.Data(172);
    %
    Status.motorState16.mode = go1_status_data.Data(173);
    Status.motorState16.q = go1_status_data.Data(174);
    Status.motorState16.dq = go1_status_data.Data(175);
    Status.motorState16.ddq = go1_status_data.Data(176);
    Status.motorState16.tauEst = go1_status_data.Data(177);
    Status.motorState16.q_raw = go1_status_data.Data(178);
    Status.motorState16.dq_raw = go1_status_data.Data(179);
    Status.motorState16.ddq_raw = go1_status_data.Data(180);
    %
    Status.motorState17.mode = go1_status_data.Data(181);
    Status.motorState17.q = go1_status_data.Data(182);
    Status.motorState17.dq = go1_status_data.Data(183);
    Status.motorState17.ddq = go1_status_data.Data(184);
    Status.motorState17.tauEst = go1_status_data.Data(185);
    Status.motorState17.q_raw = go1_status_data.Data(186);
    Status.motorState17.dq_raw = go1_status_data.Data(187);
    Status.motorState17.ddq_raw = go1_status_data.Data(188);
    %
    Status.motorState18.mode = go1_status_data.Data(189);
    Status.motorState18.q = go1_status_data.Data(190);
    Status.motorState18.dq = go1_status_data.Data(191);
    Status.motorState18.ddq = go1_status_data.Data(192);
    Status.motorState18.tauEst = go1_status_data.Data(193);
    Status.motorState18.q_raw = go1_status_data.Data(194);
    Status.motorState18.dq_raw = go1_status_data.Data(195);
    Status.motorState18.ddq_raw = go1_status_data.Data(196);
    %
    Status.motorState19.mode = go1_status_data.Data(197);
    Status.motorState19.q = go1_status_data.Data(198);
    Status.motorState19.dq = go1_status_data.Data(199);
    Status.motorState19.ddq = go1_status_data.Data(200);
    Status.motorState19.tauEst = go1_status_data.Data(201);
    Status.motorState19.q_raw = go1_status_data.Data(202);
    Status.motorState19.dq_raw = go1_status_data.Data(203);
    Status.motorState19.ddq_raw = go1_status_data.Data(204);
    %
    Status.motorState20.mode = go1_status_data.Data(205);
    Status.motorState20.q = go1_status_data.Data(206);
    Status.motorState20.dq = go1_status_data.Data(207);
    Status.motorState20.ddq = go1_status_data.Data(208);
    Status.motorState20.tauEst = go1_status_data.Data(209);
    Status.motorState20.q_raw = go1_status_data.Data(210);
    Status.motorState20.dq_raw = go1_status_data.Data(211);
    Status.motorState20.ddq_raw = go1_status_data.Data(212);
    %
    Status.footPosition2body.leg1.x = go1_status_data.Data(213);
    Status.footPosition2body.leg1.y = go1_status_data.Data(214);
    Status.footPosition2body.leg1.z = go1_status_data.Data(215);
    Status.footPosition2body.leg2.x = go1_status_data.Data(216);
    Status.footPosition2body.leg2.y = go1_status_data.Data(217);
    Status.footPosition2body.leg2.z = go1_status_data.Data(218);
    Status.footPosition2body.leg3.x = go1_status_data.Data(219);
    Status.footPosition2body.leg3.y = go1_status_data.Data(220);
    Status.footPosition2body.leg3.z = go1_status_data.Data(221);
    Status.footPosition2body.leg4.x = go1_status_data.Data(222);
    Status.footPosition2body.leg4.y = go1_status_data.Data(223);
    Status.footPosition2body.leg4.z = go1_status_data.Data(224);
    %
    Status.footSpeed2body.leg1.x = go1_status_data.Data(225);
    Status.footSpeed2body.leg1.y = go1_status_data.Data(226);
    Status.footSpeed2body.leg1.z = go1_status_data.Data(227);
    Status.footSpeed2body.leg2.x = go1_status_data.Data(228);
    Status.footSpeed2body.leg2.y = go1_status_data.Data(229);
    Status.footSpeed2body.leg2.z = go1_status_data.Data(230);
    Status.footSpeed2body.leg3.x = go1_status_data.Data(231);
    Status.footSpeed2body.leg3.y = go1_status_data.Data(232);
    Status.footSpeed2body.leg3.z = go1_status_data.Data(233);
    Status.footSpeed2body.leg4.x = go1_status_data.Data(234);
    Status.footSpeed2body.leg4.y = go1_status_data.Data(235);
    Status.footSpeed2body.leg4.z = go1_status_data.Data(236);


end