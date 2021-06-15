clc; clear;
sattelitesCount = 63; % количество спутников
satOnOrb = 9;
orbCount = 7;
antennaCount = 6;
contab = [
    2 9 55 56 18 10; %ok
    1 3 56 57 10 11; %ok
    2 4	57 58 11 12; %ok
    3 5	12 13 58 59; %ok
    4 6	13 14 59 60; %ok
    5 7	14 15 60 61; %ok
    6 8	15 16 61 62; %ok
    7 9	16 17 62 63; %ok
    8 1	17 18 63 55; %ok
    11 18 1	2 27 19; %ok
    10 12 2	3 19 20; %ok
    11 13 3	4 20 21; %ok
    12 14 4	5 21 22; %ok
    13 15 5	6 22 23; %ok
    14 16 6	7 23 24; %ok
    15 17 7	8 24 25; %ok
    16 18 8	9 25 26; %ok
    17 10 9	1 26 27; %ok
    20 27 10 11 36 28; %ok
    19 21 11 12	28 29; %ok
    20 22 12 13 29 30; %ok
    21 23 13 14 30 31; %ok
    22 24 14 15 31 32; %ok
    23 25 15 16 32 33; %ok
    24 25 16 17	33 34; %ok
    25 27 17 18	34 35; %ok
    26 19 18 10	35 36; %ok
    36 29 19 20	45 37; %ok
    28 30 20 21 37 38; %ok
    29 31 21 22 38 39; %ok
    30 32 39 40 22 23; %ok
    31 33 23 24 40 41; %ok
    32 34 41 42	24 25; %ok
    33 35 42 43	25 26; %ok
    34 36 26 27	43 44; %ok
    35 28 27 19	44 45; %ok
    38 45 28 29	54 46; %ok???
    37 39 29 30	46 47; %ok
    38 40 30 31	47 48; %ok
    39 41 31 32	48 49; %ok
    40 42 32 33	49 50; %ok
    41 43 33 34	50 51; %ok
    42 44 34 35	51 52; %ok
    43 45 52 53	35 36; %ok
    44 37 36 28	53 54; %ok
    47 54 37 38	63 55; %ok
    46 48 38 39	55 56; %ok
    47 49 39 40	56 57; %ok
    48 50 40 41	57 58; %ok
    49 51 41 42	58 59; %ok
    50 52 42 43	59 60; %ok
    51 53 43 44	60 61; %ok
    52 54 44 45	61 62; %ok
    53 46 45 37	62 63; %ok
    63 56 46 47	9 1; %ok
    55 57 47 48	1 2; %ok
    56 58 48 49	2 3; %ok
    57 59 49 50	3 4; %ok
    58 60 50 51	4 5; %ok
    59 61 51 52	5 6; %ok
    60 62 52 53	6 7; %ok
    61 63 53 54	7 8; %ok
    62 55 54 46	8 9; %ok
    ];

contab

betan = 85 * (pi / 180);
Rv = 781 * tan(betan);
isys = 48;

usys = zeros(1, satOnOrb);
uStep = 28.57;
uAccumulator = 0;
for satOne = 1:satOnOrb
    usys(satOne) = uAccumulator;
    uAccumulator = uAccumulator + uStep;
end

omsys = zeros(1, orbCount);
omStep = 180 / orbCount;
% omStep = 51.3;
omAccumulator = 0;
for orbOne = 1:orbCount
    omsys(orbOne) = omAccumulator;
    omAccumulator = omAccumulator + omStep;
end

usys
omsys

xy = zeros(3, sattelitesCount);
Grafsp = zeros(sattelitesCount, 2);
q = 1; nv = 1;

for m = 1:sattelitesCount
    xy(1, m) = cos(usys(nv) * (pi / 180)) * cos(omsys(q) * (pi / 180)) - sin(usys(nv) * (pi / 180)) * sin(omsys(q) * (pi / 180)) * cos(isys * (pi / 180));
    xy(2, m) = cos(usys(nv) * (pi / 180)) * sin(omsys(q) * (pi / 180)) - sin(usys(nv) * (pi / 180)) * cos(omsys(q) * (pi / 180)) * cos(isys * (pi / 180));
    xy(3, m) = sin(usys(nv) * (pi / 180)) * sin(isys * (pi / 180));
    Grafsp(m, 1) = asin(xy(3, m) / sqrt(xy(1, m) * xy(1, m) + xy(2, m) * xy(2, m) + xy(3, m) * xy(3, m))) * (180 / pi);
    Grafsp(m, 2) = atan2(xy(2, m), xy(1, m));

    while (Grafsp(m, 2) < -pi)
        Grafsp(m, 2) = pi * 2 + Grafsp(m, 2);
    end

    Grafsp(m, 2) = Grafsp(m, 2) * (180 / pi) + 180;
    nv = nv + 1;

    if (mod(m + 1, satOnOrb) == 0) && (m + 1 ~= sattelitesCount)
        q = q + 1; nv = 1;
    end

    if (mod(m + 1, satOnOrb) == 0)
        nv = 1;
    end

end

modelTact = 200; % imitation beats count
packagesFromEarthByTact = 120;
pksk = zeros(1, modelTact);
ppk = zeros(1, modelTact);
p = 4000000;
sred = 200;

for kl = 1:sred
    localTacts = modelTact;
    minRouteWeight = ones(1, sattelitesCount); %graph vertex weights array
    next = ones(1, sattelitesCount); % next empty points in queue
    send = zeros(1, sattelitesCount); % last package for sending array
    queueArr = zeros(sattelitesCount, localTacts * sattelitesCount, sattelitesCount); % main 3D array of sattelite system
    antenna = zeros(1, antennaCount); %antennas array
    usredp = zeros(1, localTacts);
    sredpack = zeros(1, localTacts);
    pk = 0;
    expocket = zeros(1, localTacts);
    pocket = 0;
    ds = zeros(1, sattelitesCount);
    s = distributionController(packagesFromEarthByTact * 2, sattelitesCount, minRouteWeight, Grafsp, Rv, p);

    for i = 1:packagesFromEarthByTact

        if (s(1, i) ~= s(2, i))
            routerInstance = router(sattelitesCount, contab, minRouteWeight, s(1, i), s(2, i));

            for j = 2:length(routerInstance)
                queueArr(routerInstance(1, 1), next(routerInstance(1, 1)), j - 1) = routerInstance(1, j);
            end

            next(routerInstance(1, 1)) = next(routerInstance(1, 1)) + 1;
            minRouteWeight(routerInstance(1, 1)) = minRouteWeight(routerInstance(1, 1)) + 1;
            send(routerInstance(1, 1)) = send(routerInstance(1, 1)) + 1;
        end

        if (s(1, i) == s(2, i))
            ds(s(1, i)) = ds(s(1, i)) + 1;
        end

    end

    ppk(1) = pk;
    tact = 2; % current tact
    isNotEmpty = true;

    while tact <= localTacts
        pk = 0;

        for ps = 1:sattelitesCount

            if ds(ps) ~= 0
                if ds(ps) <= p
                    pocket = pocket + ds(ps);
                    pk = pk + ds(ps);
                    ds(ps) = 0;
                else
                    pocket = pocket + p;
                    pk = pk + p;
                    ds(ps) = ds(ps) - p;
                end

            end

        end

        % sending packages from earth to sattelites
        s = distributionController(packagesFromEarthByTact * 2, sattelitesCount, minRouteWeight, Grafsp, Rv, p);

        for i = 1:packagesFromEarthByTact

            if (s(1, i) ~= s(2, i))
                routerInstance = router(sattelitesCount, contab, minRouteWeight, s(1, i), s(2, i));

                for j = 2:length(routerInstance)
                    queueArr(routerInstance(1, 1), next(routerInstance(1, 1)), j - 1) = routerInstance(1, j);
                end

                next(routerInstance(1, 1)) = next(routerInstance(1, 1)) + 1;
                minRouteWeight(routerInstance(1, 1)) = minRouteWeight(routerInstance(1, 1)) + 1;
            end

            if (s(1, i) == s(2, i))
                ds(s(1, i)) = ds(s(1, i)) + 1;
            end

        end

        % sending packages from sattelite to other wired sattelites
        for z = 1:sattelitesCount

            if (send(z) ~= 0)
                t = 1; antenna = zeros(1, antennaCount); an = 0; isNotEmpty = true;

                while isNotEmpty
                    g = queueArr(z, t, 1);
                    an = 0; in = true;

                    while ((an < antennaCount) && (in))
                        an = an + 1;

                        if contab(z, an) == g
                            in = false;
                        end

                    end

                    if (antenna(an) == 0)
                        gg = queueArr(z, t, 1);
                        queueArr(z, t, 1) = 0;
                        ind2 = true; j = 2;

                        if (queueArr(z, t, 2) == 0)
                            ds(gg) = ds(gg) + 1;
                            ind2 = false;
                            minRouteWeight(z) = minRouteWeight(z) - 1;
                            send(z) = send(z) - 1;
                        end

                        if (queueArr(z, t, 2) ~= 0)
                            next(g) = next(g) + 1;
                            send(z) = send(z) - 1;
                            minRouteWeight(g) = minRouteWeight(g) + 1;
                            minRouteWeight(z) = minRouteWeight(z) - 1;
                        end

                        while ind2 % package sending
                            queueArr(g, next(g) - 1, j - 1) = queueArr(z, t, j);
                            queueArr(z, t, j) = 0;
                            j = j + 1;

                            if ((queueArr(z, t, j) == 0) || (j > 2 * sattelitesCount))
                                ind2 = false;
                            end

                        end

                        antenna(an) = 1;
                    end

                    an = antenna(1) + antenna(2) + antenna(3) + antenna(4) + antenna(5) + antenna(6);

                    if an == antennaCount
                        isNotEmpty = false;
                    end

                    if (send(z) == 0)
                        isNotEmpty = false;
                    end

                    t = t + 1;

                    if queueArr(z, t, 1) == 0
                        isNotEmpty = false;
                    end

                end

            end

        end

        % queue sorting
        for z = 1:sattelitesCount
            isNotEmpty = true; t = 1;

            if (minRouteWeight(z) == 1) % check queue emptiness
                isNotEmpty = false;
            end

            if (minRouteWeight(z) == 2) && (queueArr(z, 1, 1) ~= 0) % queue dont need sorting
                isNotEmpty = false;
            end

            movedPackageCount = 0;

            while isNotEmpty % sorting process

                if (queueArr(z, t, 1) == 0) && (t < (next(z) - 1))
                    ind3 = true; x = t;

                    while ind3

                        if queueArr(z, x + 1, 1) ~= 0
                            ind2 = true; j = 1;

                            while ind2
                                c = queueArr(z, t, j);
                                queueArr(z, t, j) = queueArr(z, x + 1, j);
                                queueArr(z, x + 1, j) = c;
                                j = j + 1;

                                if ((queueArr(z, x + 1, j) == 0) || (j == sattelitesCount))
                                    ind2 = false;
                                end

                            end

                            ind3 = false;
                            movedPackageCount = movedPackageCount + 1;
                        end

                        x = x + 1;
                    end

                end

                if (movedPackageCount == (minRouteWeight(z) - 1)) || (t == (next(z)))
                    isNotEmpty = false;
                    send(z) = t;
                end

                t = t + 1;
            end

        end

        %----------------------------------------------------------------------------------
        % высчитываем сколько на каком спутнике пакетов и выставляем
        % vesi,send,next
        for i = 1:sattelitesCount
            isNotEmpty = true; t = 1;

            while isNotEmpty

                if (queueArr(i, t, 1) == 0)
                    isNotEmpty = false;
                end

                t = t + 1;
            end

            next(i) = t - 1;
            send(i) = next(i) - 1;
            minRouteWeight(i) = next(i);
        end

        ppk(tact) = ppk(tact) + pk;
        expocket(tact) = pocket;
        pksk(tact) = pksk(tact) + pk;
        pk = 0;
        tact = tact + 1;
    end
end

for kl = 1:localTacts
    ppk(kl) = ((ppk(kl) / sred));
end

packagesOut = ppk(localTacts) %pksk(localTacts)
packagesOutToIn = packagesOut / packagesFromEarthByTact %spack(localTacts)
figure(2)
plot(ppk)
title('Усредненное')
xlabel('Такты')
ylabel('Количество вышедших пакетов на такте')
figure(3)
bar(send)
xlabel('№ спутника')
ylabel('Пакетов в очереди')
xlim([1, sattelitesCount])
