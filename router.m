function f = router(sattelitesCount, contab, queue, Tstart, Tfinish)
    mins = zeros(1, sattelitesCount); 
    minRouteWeight = zeros(1, sattelitesCount);
    wave = zeros(1, sattelitesCount); 
    % queue = ones(1,sattelitesCount);
    minRouteWeight(Tstart) = queue(Tstart);
    indic = false; c = 1; indic2 = false;

    for i = 1:6
        minRouteWeight(contab(Tstart, i)) = queue(Tstart) + queue(contab(Tstart, i));
        wave(contab(Tstart, i)) = 1;
        mins(contab(Tstart, i)) = Tstart;

        if (contab(Tstart, i) == Tfinish)
            indic = true;
            end;
            c = c + 1;
        end

        r = 2;

        if (indic == false)

            while (indic2 == false)
                x = 0;

                for z = 1:sattelitesCount

                    if ((wave(z) == r - 1) && (z ~= Tfinish))
                        p = r;

                        for k = 1:6

                            if (minRouteWeight(contab(z, k)) == 0)
                                c = c + 1;
                                end;
                                g = minRouteWeight(z) + queue(contab(z, k));

                                if ((minRouteWeight(contab(z, k)) > g) || (minRouteWeight(contab(z, k)) == 0))
                                    minRouteWeight(contab(z, k)) = g;
                                    mins(contab(z, k)) = z;
                                    x = 1;
                                    wave(contab(z, k)) = p;
                                end

                            end

                        end

                    end

                    if(x == 0)
                    indic2 = true;

                end

                r = r + 1;
            end

            end;
            sp = zeros(1, 3);
            indic2 = false;
            t = 1;
            sp(t) = Tfinish;

            if (indic == false)

                while (indic2 == false)
                    sp(t + 1) = mins(sp(t));
                    t = t + 1;

                    if (sp(t) == Tstart)
                        indic2 = true;
                    end

                end

                spor = zeros(1, 3); k = 1;

                for i = 1:length(sp)
                    spor(i) = sp(length(sp) - i + 1);

                end

            else
                spor = [Tstart, Tfinish];
            end

            f = spor;
