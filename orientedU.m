function U=orientedU(Fbody,sig_hat,m)

U=((mrpTOdcm(sig_hat))')*[0;0;1];
U=(Fbody/m)*U;


end