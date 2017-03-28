mkdir "json"
FILES=csv/*.csv
for f in $FILES
do
  filename="${f%.*}"
  echo "Converting $f to $filename.json"
  `pandoc $f -t HTML -o $filename.json --no-highlight --mathjax`
  `tail -n+5 $f | perl -pe 's/\n/$1],[/' | (echo "[[" && cat) | perl -pe 's/,\[### End of scan/\]\n\n/m' > $filename.json`
  # rm $f
done
echo "Moving everyone to /json"
`mv csv/*.json json`
echo "Done"