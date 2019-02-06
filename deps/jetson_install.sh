julia --project -e 'using Pkg; Pkg.instantiate()'
sudo chmod +x ./libarpack.so.2.0.0
for a in "$(ls -d -1 $HOME/.julia/packages/Arpack/**/deps/usr/lib/)"; do cp -f ./libarpack.so.2.0.0 $a/; done;
julia --project -e 'using Pkg; Pkg.build(); exit()'
