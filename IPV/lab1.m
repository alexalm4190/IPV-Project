J=repmat((1:639),479,1);
I=repmat((1:479)',1,639);
I1=[I(:), I(:), I(:)+1];
J1=[J(:), J(:)+1, J(:)+1];
ind1=(J1-1)*480+I1;

%%
zs=xyz(ind1(:,1),3).*xyz(ind1(:,2),3) .* xyz(ind1(:,3),3);
ind1ok=find(zs>0);
trimesh(ind1(ind1ok,:),xyz(:,1),xyz(:,2),xyz(:,3))