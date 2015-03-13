function dets = detect_mugs(I)
I = im2uint8(I);

model = load('../data/svm_model.mat');
model = model.model;

d = load('../data/pixelmean.mat');
PIXEL_MEAN = d.imagem;

para=getPara();
Imsk = 0; % or your own mask
[rIm, cIm,~]=size(I);

% compute edges
edgesC = getEdgesC(I, para);

% get patch sizes
patchList = para.torque.defaultScale;
patchListXY = [floor(patchList./2)', floor(patchList./2)'];
% process entire image - set this to a mask to process a particular region
% (faster)
pixMask=ones(rIm, cIm);

%% get Torque volume
torqueVolume = computeTorqueVolumeXY_fast(edgesC, Imsk, patchListXY, ...
    pixMask, 2, 0, para.verbose);

% resize accordingly
torqueVolume = torqueVolume(1:para.torque.resizeFactor:end,1:para.torque.resizeFactor:end,:);

% get value map
[scaleMap, torqueFinal, torqueMinValueMap, torqueMaxValueMap] = computeScaleValueMap( torqueVolume, para.torque.defaultScale);

regions = {};
boxes = [];
n = 30;
for i = 1:n
    [y, x] = ind2sub(size(torqueFinal), torqueMaxValueMap(i));
    s = scaleMap(y, x);
    regions{i} = imcrop(I, [x*2-s/2 y*2-s/2 s s]);
    boxes = [boxes; [x*2-s/2 y*2-s/2 x*2+s/2 y*2+s/2]];
end
for i = 1:n
    [y, x] = ind2sub(size(torqueFinal), torqueMinValueMap(i));
    s = -scaleMap(y, x);
    regions{n + i} = imcrop(I, [x*2-s/2 y*2-s/2 s s]);
    boxes = [boxes; [x*2-s/2 y*2-s/2 x*2+s/2 y*2+s/2]];
end

% prepare input
batch_size = 10;
num_images = length(regions);
num_batches = ceil(length(regions)/batch_size);
initic=tic;
features = [];
labels = [];
for bb = 1 : num_batches
    batchtic = tic;
    range = 1+batch_size*(bb-1):min(num_images,batch_size * bb);
    tic
    input_data = prepare_mug_batch_im(regions(range),PIXEL_MEAN,batch_size);
    toc, tic
    fprintf('Batch %d out of %d %.2f%% Complete ETA %.2f seconds\n',...
        bb,num_batches,bb/num_batches*100,toc(initic)/bb*(num_batches-bb));
    output_data = caffe('forward', {input_data});
    toc
    output_data = squeeze(output_data{1});
    output_data = output_data(:,mod(range-1,batch_size)+1);
    features = [features; sparse(double(output_data'))];
    toc(batchtic)
end
toc(initic);

[labels, ~, scores] = predict(double(ones(size(features, 1), 1)), features, model);

% apply nms
thresh = 0.25;
raw_det_i = find(scores > thresh);
scored_boxes = cat(2, boxes(raw_det_i, :), scores(raw_det_i));
keep = nms(scored_boxes, 0.15);
dets = scored_boxes(keep, :);
dets = dets(:, 1:4);
