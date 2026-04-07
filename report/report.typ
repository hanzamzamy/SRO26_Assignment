#set text(font: "New Computer Modern", size: 10pt, lang: "id")
#set par(justify: true, first-line-indent: (all: true, amount: 1.8em))
#set heading(numbering: "1.1.")
#set math.equation(numbering: "(1)")
#set page(
  paper: "a4",
  margin: (left: 20mm, right: 20mm, top: 35mm, bottom: 20mm),
  header-ascent: 12mm,
  header: context {
    stack(
      dir: ttb,
      grid(
        columns: (1fr, 1fr),
        align(left)[#image("ITS_pojok.pdf", width: 2cm)],
        align(right)[#text(size: 10pt)[#datetime.today().display("[year]-[month]-[day]")]]
      ),
      v(0.4em),
      line(length: 100%, stroke: 0.5pt)
    )
  }
)

// Bagian Judul
#align(center)[
  #v(1em)
  #text(size: 18pt)[Laporan Analisis Spasial dan Odometri Pioneer P3DX] 
  #v(1em)
]

// Tabel Metadata
#grid(
  columns: (auto, 1fr),
  gutter: 1em,
  [*Penulis*], [: Rayhan Rizqi Zamzamy],
  [*Dosen*], [: Muhammad Qomaruz Zaman, S.T., M.T., Ph.D.],
  [*Nama Kelas*], [: Sistem Robot Otonom],
  [*Link video YouTube*], [: #link("https://youtu.be/xgot_TmPZvw")],
  [*Link GitHub*], [: #link("https://github.com/hanzamzamy/SRO26_Assignment/tree/5")]
)
#v(1.5cm)

= Pendahuluan
Laporan ini membahas perbandingan profil lintasan spasial 2D dari robot _differential drive_ Pioneer P3DX menggunakan tiga pendekatan berbeda: integrasi odometri sudut relatif, odometri sudut absolut (orientasi _ground truth_), dan lintasan aktual di dalam simulator. Analisis ini bertujuan untuk mengevaluasi dampak slip roda dan akumulasi eror pada kalkulasi posisi (odometri) _mobile robot_ @tzafestas2013introduction. 

= Pemodelan Odometri
Kalkulasi posisi robot dilakukan dengan mengintegrasikan kecepatan translasi ($V_x$) terhadap waktu ($Delta t$). Kecepatan translasi didapatkan dari persamaan _forward kinematics_ dasar berdasarkan kecepatan sudut roda kanan dan kiri @corke2011robotics. Dalam analisis ini, terdapat dua metode kalkulasi lintasan yang digunakan. Persamaan kinematika untuk kecepatan translasi dan angular adalah sebagai berikut.

$ V_x = (r/2) (omega_r + omega_l) $ <eq:vx>
$ omega_x = (r/L) (omega_r - omega_l) $ <eq:omega>

#h(-1.8em) Dimana $r$ adalah jari-jari roda, $L$ adalah jarak antar roda, $omega_r$, & $omega_l$ adalah kecepatan sudut roda kanan & kiri.

== Odometri Sudut Relatif (Integrasi Kecepatan Sudut)
Metode ini murni bergantung pada data internal dari perputaran roda aktuator (_proprioceptive sensors_). Sudut _heading_ atau orientasi bodi robot ($theta$) diestimasi dengan mengintegrasikan kecepatan sudut ($omega_x$) dari persamaan kinematika.
$ theta_(k) = theta_(k-1) + omega_x Delta t $ <eq:theta_rel>
$ x_k = x_(k-1) + V_x cos(theta_k) Delta t $ <eq:x_rel>
$ y_k = y_(k-1) + V_x sin(theta_k) Delta t $ <eq:y_rel>

== Odometri Sudut Absolut (Data Orientasi Aktual)
Metode ini menggunakan kecepatan translasi dari roda ($V_x$), namun mengabaikan estimasi orientasi dari integrasi kinematika. Sebagai gantinya, sudut _heading_ ($theta_"sim"$) ditarik langsung dari data absolut simulator sebagai representasi sensor IMU atau kompas digital yang ideal di dunia nyata.
$ x_k = x_(k-1) + V_x cos(theta_"sim") Delta t $ <eq:x_abs>
$ y_k = y_(k-1) + V_x sin(theta_"sim") Delta t $ <eq:y_abs>

= Analisis Perbandingan Lintasan Spasial
Profil lintasan selama simulasi 90 detik di-_plot_ ke dalam bidang kartesian 2D, menghasilkan tiga kurva spasial yang merepresentasikan ketiga metode pembacaan posisi.

#figure(
  image("odom_spatial.png", width: 80%),
  caption: [Posisi X-Y P3DX metode odometri dan _ground truth_.],
)

#h(-1.8em) Berdasarkan _plot_ di atas, terdapat beberapa temuan krusial terkait sifat dasar odometri dan simulasi fisika.

+ *Perbedaan Titik Awal (_Reference Frame_):* Lintasan _Ground Truth_ (hitam) dimulai dari titik koordinat aktual bodi robot di dalam ruang simulasi absolut $(0.6, -0.125)$. Sebaliknya, kedua lintasan odometri (hijau dan biru) dimulai dari titik $(0, 0)$. Hal ini merepresentasikan sifat alami odometri yang bekerja pada kerangka referensi lokal (_relative frame_), di mana posisi dihitung relatif terhadap titik mula, bukan terhadap koordinat dunia nyata.
+ *Lintasan _Ground Truth_ (Hitam):* Lintasan ini menunjukkan posisi aktual (_center of mass_) robot di dalam simulasi CoppeliaSim. Terlihat bahwa robot bergerak menyusuri area di dalam kotak batas merah ($5 "m" times 5 "m"$) dan berhasil menghindari rintangan (dinding) secara konsisten pada jarak tertentu karena dipandu oleh algoritma Braitenberg.
+ *Deviasi Odometri Relatif (Hijau):* Lintasan hijau yang dikalkulasi murni dari integrasi putaran roda mengalami divergensi yang sangat parah. Karena roda robot mengalami slip fisik (slip translasi dan slip rotasi akibat tarikan _caster wheel_), kecepatan aktuator roda tidak 100% terkonversi menjadi pergerakan bodi. Eror kecil pada integrasi $omega$ (@eq:theta_rel) menyebabkan eror orientasi orientasi. Eror pada _heading_ ini terakumulasi terus menerus seiring waktu (_unbounded cumulative error_), membelokkan arah integrasi sumbu $X$ dan $Y$ hingga akhirnya lintasan bergeser secara masif.
+ *Koreksi Odometri Absolut (Biru):* Lintasan biru menggunakan orientasi yang sempurna ($theta_"sim"$). Akibatnya, profil lintasan biru memiliki bentuk pola haluan yang sangat mirip dan sejajar dengan lintasan _Ground Truth_. Hal ini membuktikan bahwa menghilangkan eror integrasi orientasi dapat secara signifikan menekan deviasi lintasan (_drift_). Meskipun demikian, masih terdapat deviasi linear dibandingkan lintasan _Ground Truth_. Hal ini disebabkan karena nilai ($V_x$) masih ditarik dari integrasi putaran roda yang mengalami slip, sehingga jarak tempuh robot menurut putaran roda sedikit berbeda dari jarak tempuh bodi fisiknya.

#figure(
  image("spatial_same_origin.png", width: 80%),
  caption: [Posisi X-Y P3DX dengan titik awal sama $(0, 0)$.],
)

#figure(
  image("spatial_different_orientation.png", width: 80%),
  caption: [Posisi X-Y P3DX dengan titik awal sama, orientasi berbeda ($90degree$).],
)

Beberapa kasus tambahan juga dianalisis untuk memvalidasi temuan utama. Pada kasus pertama, semua lintasan di-_plot_ dengan titik awal yang sama $(0, 0)$ untuk menegaskan _drift_ pada metode odometri. Metode sudut absolut menunjukan hasil yang sangat mendekati lintasan _Ground Truth_, sementara metode sudut relatif tetap menunjukkan deviasi yang signifikan. Pada kasus kedua, orientasi awal dari semua metode disamakan ($90degree$) pada posisi awal $(0, 0)$ untuk menegaskan _relative frame_ dari odometri. Metode sudut relatif memiliki pose yang berbeda, yaitu sudut awal diasumsikan $0degree$, karena estimasi berdasarkan pada $omega_x$. Metode sudut absolut memiliki pose yang sama dengan lintasan _Ground Truth_ karena menggunakan orientasi aktual dari simulator.

= Kesimpulan
_Plot_ spasial ini membuktikan bahwa kalkulasi odometri relatif murni menggunakan kecepatan roda sangat rentan terhadap akumulasi eror, khususnya pada estimasi _heading_ akibat slip mekanis yang diperparah seiring berjalannya waktu. Penggunaan data orientasi eksternal absolut (_Absolute Angle Odometry_) terbukti sukses mengoreksi penyimpangan lintasan, mempertahankan bentuk manuver asli robot, meskipun kompensasi translasi (_wheel slip translation_) tetap dibutuhkan untuk mencapai akurasi _ground truth_ seutuhnya.

#bibliography("references.bib", style: "ieee")