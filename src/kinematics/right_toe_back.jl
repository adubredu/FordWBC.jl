function right_toe_back_helper!(p_output1, var1)
  t566 = cos(var1[3+1])
  t761 = cos(var1[5+1])
  t793 = sin(var1[3+1])
  t765 = sin(var1[4+1])
  t803 = sin(var1[5+1])
  t620 = cos(var1[19+1])
  t629 = -1.0 * t620
  t677 = 1.0 +  t629
  t716 = sin(var1[19+1])
  t750 = sin(var1[18+1])
  t766 = t566*t761*t765
  t807 = t793*t803
  t808 = t766 + t807
  t830 = cos(var1[18+1])
  t835 = -1.0 * t761*t793
  t838 = t566*t765*t803
  t851 = t835 + t838
  t598 = cos(var1[4+1])
  t883 = -1.0 * t750*t808
  t890 = t830*t851
  t903 = t883 + t890
  t921 = t830*t808
  t923 = t750*t851
  t931 = t921 + t923
  t945 = cos(var1[20+1])
  t946 = -1.0 * t945
  t947 = 1.0 +  t946
  t958 = sin(var1[20+1])
  t977 = -0.366501*t566*t598*t716
  t978 = 0.340999127418*t677*t903
  t989 = -0.134322983001*t677
  t990 = 1.0 +  t989
  t995 = t990*t931
  t996 = t977 + t978 + t995
  t1016 = 0.930418*t566*t598*t716
  t1017 = -0.8656776547239999*t677
  t1018 = 1.0 +  t1017
  t1026 = t1018*t903
  t1031 = 0.340999127418*t677*t931
  t1032 = t1016 + t1026 + t1031
  t1049 = -1.000000637725*t677
  t1051 = 1.0 +  t1049
  t1057 = t1051*t566*t598
  t1062 = -0.930418*t716*t903
  t1069 = 0.366501*t716*t931
  t1070 = t1057 + t1062 + t1069
  t1077 = cos(var1[21+1])
  t1084 = -1.0 * t1077
  t1092 = 1.0 +  t1084
  t1099 = sin(var1[21+1])
  t1103 = 0.930418*t958*t996
  t1113 = 0.366501*t958*t1032
  t1114 = -1.000000637725*t947
  t1117 = 1.0 +  t1114
  t1118 = t1117*t1070
  t1123 = t1103 + t1113 + t1118
  t1152 = -0.8656776547239999*t947
  t1154 = 1.0 +  t1152
  t1155 = t1154*t996
  t1163 = -0.340999127418*t947*t1032
  t1164 = -0.930418*t958*t1070
  t1173 = t1155 + t1163 + t1164
  t1196 = -0.340999127418*t947*t996
  t1197 = -0.134322983001*t947
  t1201 = 1.0 +  t1197
  t1202 = t1201*t1032
  t1211 = -0.366501*t958*t1070
  t1215 = t1196 + t1202 + t1211
  t1228 = cos(var1[22+1])
  t1230 = -1.0 * t1228
  t1231 = 1.0 +  t1230
  t1240 = sin(var1[22+1])
  t1246 = 0.366501*t1099*t1123
  t1247 = -0.340999127418*t1092*t1173
  t1255 = -0.134322983001*t1092
  t1256 = 1.0 +  t1255
  t1258 = t1256*t1215
  t1259 = t1246 + t1247 + t1258
  t1269 = 0.930418*t1099*t1123
  t1276 = -0.8656776547239999*t1092
  t1277 = 1.0 +  t1276
  t1278 = t1277*t1173
  t1279 = -0.340999127418*t1092*t1215
  t1280 = t1269 + t1278 + t1279
  t1288 = -1.000000637725*t1092
  t1291 = 1.0 +  t1288
  t1292 = t1291*t1123
  t1295 = -0.930418*t1099*t1173
  t1296 = -0.366501*t1099*t1215
  t1301 = t1292 + t1295 + t1296
  t1303 = cos(var1[23+1])
  t1304 = -1.0 * t1303
  t1305 = 1.0 +  t1304
  t1309 = sin(var1[23+1])
  t1320 = -0.366501*t1240*t1259
  t1321 = -0.930418*t1240*t1280
  t1323 = -1.000000637725*t1231
  t1324 = 1.0 +  t1323
  t1331 = t1324*t1301
  t1332 = t1320 + t1321 + t1331
  t1338 = -0.134322983001*t1231
  t1339 = 1.0 +  t1338
  t1340 = t1339*t1259
  t1341 = -0.340999127418*t1231*t1280
  t1344 = 0.366501*t1240*t1301
  t1345 = t1340 + t1341 + t1344
  t1354 = -0.340999127418*t1231*t1259
  t1355 = -0.8656776547239999*t1231
  t1356 = 1.0 +  t1355
  t1357 = t1356*t1280
  t1358 = 0.930418*t1240*t1301
  t1359 = t1354 + t1357 + t1358
  t1362 = cos(var1[24+1])
  t1365 = -1.0 * t1362
  t1366 = 1.0 +  t1365
  t1368 = sin(var1[24+1])
  t1421 = cos(var1[25+1])
  t1424 = -1.0 * t1421
  t1427 = 1.0 +  t1424
  t1431 = sin(var1[25+1])
  t1371 = 0.930418*t1309*t1332
  t1373 = -0.340999127418*t1305*t1345
  t1374 = -0.8656776547239999*t1305
  t1375 = 1.0 +  t1374
  t1376 = t1375*t1359
  t1378 = t1371 + t1373 + t1376
  t1387 = 0.366501*t1309*t1332
  t1389 = -0.134322983001*t1305
  t1391 = 1.0 +  t1389
  t1393 = t1391*t1345
  t1394 = -0.340999127418*t1305*t1359
  t1395 = t1387 + t1393 + t1394
  t1408 = -1.000000637725*t1305
  t1409 = 1.0 +  t1408
  t1411 = t1409*t1332
  t1414 = -0.366501*t1309*t1345
  t1415 = -0.930418*t1309*t1359
  t1418 = t1411 + t1414 + t1415
  t1438 = -0.175248972904*t1427
  t1467 = -0.120666640478*t1427
  t1430 = -0.444895486988*t1427
  t1488 = 0.218018*t1431
  t1448 = -0.930418*t1368*t1378
  t1450 = -0.366501*t1368*t1395
  t1453 = -1.000000637725*t1366
  t1454 = 1.0 +  t1453
  t1455 = t1454*t1418
  t1456 = t1448 + t1450 + t1455
  t1485 = -0.803828*t1431
  t1473 = -0.340999127418*t1366*t1378
  t1475 = -0.134322983001*t1366
  t1476 = 1.0 +  t1475
  t1477 = t1476*t1395
  t1478 = 0.366501*t1368*t1418
  t1480 = t1473 + t1477 + t1478
  t1493 = -0.8656776547239999*t1366
  t1495 = 1.0 +  t1493
  t1496 = t1495*t1378
  t1497 = -0.340999127418*t1366*t1395
  t1498 = 0.930418*t1368*t1418
  t1500 = t1496 + t1497 + t1498
  t1443 = 0.553471*t1431
  t1503 = 0.444895486988*t1427
  t1433 = -0.218018*t1431
  t1518 = 0.175248972904*t1427
  t1463 = -0.553471*t1431
  t1507 = 0.120666640478*t1427
  t1468 = 0.803828*t1431
  t709 = -0.04500040093286238*t677
  t745 = 0.0846680539949003*t716
  t746 = t709 + t745
  t831 = -1.0 * t830
  t834 = 1.0 +  t831
  t869 = 1.296332362046933e-7*var1[19+1]
  t872 = -0.07877668146182712*t677
  t874 = -0.04186915633414423*t716
  t879 = t869 + t872 + t874
  t1538 = t761*t793*t765
  t1539 = -1.0 * t566*t803
  t1540 = t1538 + t1539
  t1543 = t566*t761
  t1545 = t793*t765*t803
  t1546 = t1543 + t1545
  t907 = 3.2909349868922137e-7*var1[19+1]
  t916 = 0.03103092645718495*t677
  t919 = 0.016492681424499736*t716
  t920 = t907 + t916 + t919
  t939 = -1.296332362046933e-7*var1[20+1]
  t953 = -0.14128592423750855*t947
  t959 = 0.04186915633414423*t958
  t976 = t939 + t953 + t959
  t1549 = -1.0 * t750*t1540
  t1550 = t830*t1546
  t1552 = t1549 + t1550
  t1554 = t830*t1540
  t1558 = t750*t1546
  t1559 = t1554 + t1558
  t1000 = 3.2909349868922137e-7*var1[20+1]
  t1008 = -0.055653945343889656*t947
  t1009 = 0.016492681424499736*t958
  t1012 = t1000 + t1008 + t1009
  t1041 = -0.04500040093286238*t947
  t1047 = -0.15185209683981668*t958
  t1048 = t1041 + t1047
  t1093 = 0.039853038461262744*t1092
  t1100 = 0.23670515095269612*t1099
  t1102 = t1093 + t1100
  t1562 = -0.366501*t598*t716*t793
  t1563 = 0.340999127418*t677*t1552
  t1564 = t990*t1559
  t1565 = t1562 + t1563 + t1564
  t1568 = 0.930418*t598*t716*t793
  t1569 = t1018*t1552
  t1570 = 0.340999127418*t677*t1559
  t1572 = t1568 + t1569 + t1570
  t1575 = t1051*t598*t793
  t1577 = -0.930418*t716*t1552
  t1578 = 0.366501*t716*t1559
  t1579 = t1575 + t1577 + t1578
  t1125 = 6.295460977284962e-8*var1[21+1]
  t1128 = -0.22023473313910558*t1092
  t1147 = 0.03707996069223323*t1099
  t1150 = t1125 + t1128 + t1147
  t1175 = -1.5981976069815686e-7*var1[21+1]
  t1177 = -0.08675267452931407*t1092
  t1182 = 0.014606169134372047*t1099
  t1189 = t1175 + t1177 + t1182
  t1225 = -4.0833068682577724e-7*var1[22+1]
  t1234 = -0.11476729583292707*t1231
  t1241 = 0.0111594154470601*t1240
  t1242 = t1225 + t1234 + t1241
  t1582 = 0.930418*t958*t1565
  t1583 = 0.366501*t958*t1572
  t1584 = t1117*t1579
  t1585 = t1582 + t1583 + t1584
  t1588 = t1154*t1565
  t1589 = -0.340999127418*t947*t1572
  t1590 = -0.930418*t958*t1579
  t1592 = t1588 + t1589 + t1590
  t1594 = -0.340999127418*t947*t1565
  t1595 = t1201*t1572
  t1597 = -0.366501*t958*t1579
  t1598 = t1594 + t1595 + t1597
  t1265 = 1.6084556086870008e-7*var1[22+1]
  t1266 = -0.29135406957765553*t1231
  t1267 = 0.02832985722118838*t1240
  t1268 = t1265 + t1266 + t1267
  t1284 = 0.03044854601678662*t1231
  t1286 = 0.3131431996991197*t1240
  t1287 = t1284 + t1286
  t1308 = -0.26285954081199375*t1305
  t1315 = 0.634735404786378*t1309
  t1316 = t1308 + t1315
  t1602 = 0.366501*t1099*t1585
  t1604 = -0.340999127418*t1092*t1592
  t1605 = t1256*t1598
  t1607 = t1602 + t1604 + t1605
  t1610 = 0.930418*t1099*t1585
  t1612 = t1277*t1592
  t1613 = -0.340999127418*t1092*t1598
  t1614 = t1610 + t1612 + t1613
  t1618 = t1291*t1585
  t1619 = -0.930418*t1099*t1592
  t1620 = -0.366501*t1099*t1598
  t1622 = t1618 + t1619 + t1620
  t1334 = 1.6169269214444473e-7*var1[23+1]
  t1335 = -0.2326311605896123*t1305
  t1336 = -0.09633822312984319*t1309
  t1337 = t1334 + t1335 + t1336
  t1350 = -6.369237629068993e-8*var1[23+1]
  t1351 = -0.5905692458505322*t1305
  t1352 = -0.24456909227538925*t1309
  t1353 = t1350 + t1351 + t1352
  t1361 = -7.041766963257243e-8*var1[24+1]
  t1367 = -0.8232948486053725*t1366
  t1369 = 0.05763710717422546*t1368
  t1370 = t1361 + t1367 + t1369
  t1624 = -0.366501*t1240*t1607
  t1627 = -0.930418*t1240*t1614
  t1628 = t1324*t1622
  t1629 = t1624 + t1627 + t1628
  t1632 = t1339*t1607
  t1633 = -0.340999127418*t1231*t1614
  t1634 = 0.366501*t1240*t1622
  t1635 = t1632 + t1633 + t1634
  t1638 = -0.340999127418*t1231*t1607
  t1639 = t1356*t1614
  t1640 = 0.930418*t1240*t1622
  t1642 = t1638 + t1639 + t1640
  t1383 = 1.7876586242383724e-7*var1[24+1]
  t1384 = -0.3243041141817093*t1366
  t1385 = 0.02270383571304597*t1368
  t1386 = t1383 + t1384 + t1385
  t1402 = 0.06194758047549556*t1366
  t1403 = 0.8848655643005321*t1368
  t1404 = t1402 + t1403
  t1420 = 2.7989049814696287e-7*var1[25+1]
  t1429 = 0.15748067958019524*t1427
  t1435 = t1430 + t1433
  t1436 = -0.528674719304*t1435
  t1444 = t1438 + t1443
  t1445 = -0.29871295412*t1444
  t1447 = t1420 + t1429 + t1436 + t1445
  t1644 = 0.930418*t1309*t1629
  t1645 = -0.340999127418*t1305*t1635
  t1647 = t1375*t1642
  t1648 = t1644 + t1645 + t1647
  t1650 = 0.366501*t1309*t1629
  t1652 = t1391*t1635
  t1653 = -0.340999127418*t1305*t1642
  t1654 = t1650 + t1652 + t1653
  t1656 = t1409*t1629
  t1657 = -0.366501*t1309*t1635
  t1658 = -0.930418*t1309*t1642
  t1659 = t1656 + t1657 + t1658
  t1460 = 7.591321355439789e-8*var1[25+1]
  t1461 = -0.2845150083511607*t1427
  t1465 = t1438 + t1463
  t1466 = 0.445034169498*t1465
  t1470 = t1467 + t1468
  t1471 = -0.528674719304*t1470
  t1472 = t1460 + t1461 + t1466 + t1471
  t1482 = 1.9271694180831932e-7*var1[25+1]
  t1483 = -0.3667264808254521*t1427
  t1486 = t1467 + t1485
  t1487 = -0.29871295412*t1486
  t1490 = t1430 + t1488
  t1491 = 0.445034169498*t1490
  t1492 = t1482 + t1483 + t1487 + t1491
  t1505 = t1503 + t1488
  t1661 = -0.930418*t1368*t1648
  t1662 = -0.366501*t1368*t1654
  t1663 = t1454*t1659
  t1664 = t1661 + t1662 + t1663
  t1508 = t1507 + t1485
  t1666 = -0.340999127418*t1366*t1648
  t1667 = t1476*t1654
  t1668 = 0.366501*t1368*t1659
  t1669 = t1666 + t1667 + t1668
  t1510 = -0.693671301908*t1427
  t1511 = 1.0 +  t1510
  t1671 = t1495*t1648
  t1672 = -0.340999127418*t1366*t1654
  t1674 = 0.930418*t1368*t1659
  t1675 = t1671 + t1672 + t1674
  t1515 = -0.353861996165*t1427
  t1516 = 1.0 +  t1515
  t1520 = t1518 + t1443
  t1522 = t1503 + t1433
  t1526 = t1518 + t1463
  t1528 = -0.952469601425*t1427
  t1530 = 1.0 +  t1528
  t1532 = t1507 + t1468
  t1703 = -1.0 * t598*t761*t750
  t1704 = t830*t598*t803
  t1705 = t1703 + t1704
  t1707 = t830*t598*t761
  t1708 = t598*t750*t803
  t1709 = t1707 + t1708
  t1712 = 0.366501*t716*t765
  t1713 = 0.340999127418*t677*t1705
  t1714 = t990*t1709
  t1715 = t1712 + t1713 + t1714
  t1717 = -0.930418*t716*t765
  t1718 = t1018*t1705
  t1719 = 0.340999127418*t677*t1709
  t1720 = t1717 + t1718 + t1719
  t1722 = -1.0 * t1051*t765
  t1723 = -0.930418*t716*t1705
  t1725 = 0.366501*t716*t1709
  t1726 = t1722 + t1723 + t1725
  t1728 = 0.930418*t958*t1715
  t1729 = 0.366501*t958*t1720
  t1730 = t1117*t1726
  t1732 = t1728 + t1729 + t1730
  t1734 = t1154*t1715
  t1735 = -0.340999127418*t947*t1720
  t1736 = -0.930418*t958*t1726
  t1738 = t1734 + t1735 + t1736
  t1740 = -0.340999127418*t947*t1715
  t1741 = t1201*t1720
  t1745 = -0.366501*t958*t1726
  t1746 = t1740 + t1741 + t1745
  t1749 = 0.366501*t1099*t1732
  t1752 = -0.340999127418*t1092*t1738
  t1753 = t1256*t1746
  t1754 = t1749 + t1752 + t1753
  t1757 = 0.930418*t1099*t1732
  t1758 = t1277*t1738
  t1759 = -0.340999127418*t1092*t1746
  t1760 = t1757 + t1758 + t1759
  t1762 = t1291*t1732
  t1763 = -0.930418*t1099*t1738
  t1765 = -0.366501*t1099*t1746
  t1766 = t1762 + t1763 + t1765
  t1768 = -0.366501*t1240*t1754
  t1769 = -0.930418*t1240*t1760
  t1770 = t1324*t1766
  t1771 = t1768 + t1769 + t1770
  t1773 = t1339*t1754
  t1774 = -0.340999127418*t1231*t1760
  t1775 = 0.366501*t1240*t1766
  t1776 = t1773 + t1774 + t1775
  t1778 = -0.340999127418*t1231*t1754
  t1779 = t1356*t1760
  t1780 = 0.930418*t1240*t1766
  t1781 = t1778 + t1779 + t1780
  t1783 = 0.930418*t1309*t1771
  t1784 = -0.340999127418*t1305*t1776
  t1785 = t1375*t1781
  t1786 = t1783 + t1784 + t1785
  t1789 = 0.366501*t1309*t1771
  t1790 = t1391*t1776
  t1791 = -0.340999127418*t1305*t1781
  t1793 = t1789 + t1790 + t1791
  t1795 = t1409*t1771
  t1796 = -0.366501*t1309*t1776
  t1799 = -0.930418*t1309*t1781
  t1800 = t1795 + t1796 + t1799
  t1803 = -0.930418*t1368*t1786
  t1804 = -0.366501*t1368*t1793
  t1805 = t1454*t1800
  t1806 = t1803 + t1804 + t1805
  t1809 = -0.340999127418*t1366*t1786
  t1811 = t1476*t1793
  t1812 = 0.366501*t1368*t1800
  t1813 = t1809 + t1811 + t1812
  t1816 = t1495*t1786
  t1817 = -0.340999127418*t1366*t1793
  t1818 = 0.930418*t1368*t1800
  t1819 = t1816 + t1817 + t1818
  p_output1[0+1]=t1012*t1032 + t1048*t1070 + t1102*t1123 + t1150*t1173 + t1189*t1215 + t1242*t1259 + t1268*t1280 + t1287*t1301 + t1316*t1332 + t1337*t1345 + t1353*t1359 + t1370*t1378 + t1386*t1395 + t1404*t1418 + t1447*t1456 + t1472*t1480 + t1492*t1500 - 0.861971*(t1456*t1505 + t1480*t1508 + t1500*t1511) - 0.037329*(t1456*t1516 + t1480*t1520 + t1500*t1522) - 0.430001*(t1456*t1526 + t1480*t1530 + t1500*t1532) + t566*t598*t746 - 0.091*t750*t808 - 0.091*t834*t851 + t879*t903 + t920*t931 + t976*t996 + var1[0+1]
  p_output1[1+1]=t1012*t1572 + t1048*t1579 + t1102*t1585 + t1150*t1592 + t1189*t1598 + t1242*t1607 + t1268*t1614 + t1287*t1622 + t1316*t1629 + t1337*t1635 + t1353*t1642 + t1370*t1648 + t1386*t1654 + t1404*t1659 + t1447*t1664 + t1472*t1669 + t1492*t1675 - 0.861971*(t1505*t1664 + t1508*t1669 + t1511*t1675) - 0.037329*(t1516*t1664 + t1520*t1669 + t1522*t1675) - 0.430001*(t1526*t1664 + t1530*t1669 + t1532*t1675) - 0.091*t1540*t750 + t598*t746*t793 - 0.091*t1546*t834 + t1552*t879 + t1559*t920 + t1565*t976 + var1[1+1]
  p_output1[2+1]=t1012*t1720 + t1048*t1726 + t1102*t1732 + t1150*t1738 + t1189*t1746 + t1242*t1754 + t1268*t1760 + t1287*t1766 + t1316*t1771 + t1337*t1776 + t1353*t1781 + t1370*t1786 + t1386*t1793 + t1404*t1800 + t1447*t1806 + t1472*t1813 + t1492*t1819 - 0.861971*(t1505*t1806 + t1508*t1813 + t1511*t1819) - 0.037329*(t1516*t1806 + t1520*t1813 + t1522*t1819) - 0.430001*(t1526*t1806 + t1530*t1813 + t1532*t1819) - 0.091*t598*t750*t761 - 1.0 * t746*t765 - 0.091*t598*t803*t834 + t1705*t879 + t1709*t920 + t1715*t976 + var1[2+1]
end



function right_toe_back(var1)
	p_output1 = zeros(3)
	right_toe_back_helper!(p_output1, var1)
	return p_output1 
end