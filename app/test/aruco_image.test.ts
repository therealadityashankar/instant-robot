// The sim paints these markers into the scene and the real detector reads them
// back, so a marker that renders but doesn't decode means "the camera sees
// nothing" with no error anywhere. This runs that exact round trip.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { loadCv } from '../src/lib/cv';

test('generated markers decode with the detector that reads them', async () => {
  const cv: any = await loadCv();
  const dict = cv.getPredefinedDictionary(cv.DICT_6X6_250);
  const detector = new cv.aruco_ArucoDetector(
    dict,
    new cv.aruco_DetectorParameters(),
    new cv.aruco_RefineParameters(10, 3, true),
  );

  for (const id of [200, 101]) {
    const marker = new cv.Mat();
    dict.generateImageMarker(id, 160, marker, 1);
    // A quiet zone is not optional: without light around the black border the
    // detector can't find the square at all.
    const padded = new cv.Mat();
    cv.copyMakeBorder(marker, padded, 40, 40, 40, 40, cv.BORDER_CONSTANT, new cv.Scalar(255));

    const corners = new cv.MatVector();
    const ids = new cv.Mat();
    detector.detectMarkers(padded, corners, ids);
    const found = ids.rows ? Array.from(ids.data32S as Int32Array) : [];
    assert.deepEqual(found, [id], `marker ${id} did not decode`);

    marker.delete();
    padded.delete();
    corners.delete();
    ids.delete();
  }
});
